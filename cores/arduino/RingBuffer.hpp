/*!
 * \file RingBuffer.hpp
 * \version 2.0.4
 * \brief Simple SPSC ring buffer implementation
 *
 * \author Jan Oleksiewicz <jnk0le@hotmail.com>
 * \license SPDX-License-Identifier: MIT
 * \date 22 Jun 2017
 */


#ifndef RingBuffer_HPP
#define RingBuffer_HPP

#include <stdint.h>
#include <stddef.h>
#include <limits>
#include <atomic>

#define DEFAULT_BUFFER_SIZE 256
template<size_t buffer_size = DEFAULT_BUFFER_SIZE>
class RingBuffer {
public:
    /*!
     * \brief Default constructor, will initialize head and tail indexes
     */
    RingBuffer() : head(0), tail(0) {
    }

    void clear(void) {
        tail = head;
    }

    /*!
     * \brief Check if buffer is empty
     * \return True if buffer is empty
     */
    bool isEmpty(void) const {
        return count() == 0;
    }

    /*!
     * \brief Check if buffer is full
     * \return True if buffer is full
     */
    bool isFull(void) const {
        return capacity() == 0;
    }

    /*!
     * \brief Check how many elements can be read from the buffer
     * \return Number of elements that can be read
     */
    size_t count(void) const {
        return head - tail;
    }

    /*!
     * \brief Check how many elements can be written into the buffer
     * \return Number of free slots that can be be written
     */
    size_t capacity(void) const {
        return buffer_size - (head - tail);
    }

    /*!
     * \brief pushes data into internal buffer, without blocking
     * \param data element to be pushed into internal buffer
     * \return True if data was pushed
     */
    bool push(uint8_t data) {
        if ((head - tail) == buffer_size)
            return false;
        else {
            data_buff[head++ & buffer_mask] = data;
        }
        return true;
    }

    /*!
     * \brief pushes data into internal buffer, without blocking
     * \param[in] data Pointer to memory location where element, to be pushed into internal buffer, is located
     * \return True if data was pushed
     */
    bool push(const uint8_t *data) {
        if ((head - tail) == buffer_size)
            return false;
        else {
            data_buff[head++ & buffer_mask] = *data;
        }
        return true;
    }

    /*!
     * \brief pushes data returned by callback function, into internal buffer, without blocking
     *
     * This is a special purpose function that can be used to avoid redundant availability checks in case when
     * acquiring data have a side effects (like clearing status flags by reading a peripheral data register)
     *
     * \param get_data_callback Pointer to callback function that returns element to be pushed into buffer
     * \return True if data was pushed and callback called
     */
    bool pushFromCallbackWhenAvailable(uint8_t (*get_data_callback)(void)) {
        if ((head - tail) == buffer_size)
            return false;
        else {
            // execute callback only when there is space in buffer
            data_buff[head++ & buffer_mask] = get_data_callback();
        }
        return true;
    }

    /*!
     * \brief Removes single element without reading
     * \return True if one element was removed
     */
    bool pop() {

        if (tail == head)
            return false;
        else
            tail++;
        return true;
    }

    /*!
     * \brief Removes multiple elements without reading and storing it elsewhere
     * \param cnt Maximum number of elements to remove
     * \return Number of removed elements
     */
    size_t pop(size_t cnt) {
        size_t avail = head - tail;

        cnt = (cnt > avail) ? avail : cnt;

        tail = tail + cnt;
        return cnt;
    }

    /*!
     * \brief Reads one element from internal buffer without blocking
     * \param[out] data Reference to memory location where removed element will be stored
     * \return True if data was fetched from the internal buffer
     */
    bool pop(uint8_t &data) {
        return pop(&data); // references are anyway implemented as pointers
    }

    /*!
     * \brief Reads one element from internal buffer without blocking
     * \param[out] data Pointer to memory location where removed element will be stored
     * \return True if data was fetched from the internal buffer
     */
    bool pop(uint8_t *data) {
        if (tail == head)
            return false;
        else {
            *data = data_buff[tail++ & buffer_mask];
        }
        return true;
    }

    /*!
     * \brief Gets the first element in the buffer on consumed side
     *
     * It is safe to use and modify item contents only on consumer side
     *
     * \return Pointer to first element, nullptr if buffer was empty
     */
    uint8_t *peek() {
        if (tail == head)
            return nullptr;
        else
            return &data_buff[tail & buffer_mask];
    }

    /*!
     * \brief Gets the n'th element on consumed side
     *
     * It is safe to use and modify item contents only on consumer side
     *
     * \param index Item offset starting on the consumed side
     * \return Pointer to requested element, nullptr if index exceeds storage count
     */
    uint8_t *at(size_t index) {
        if ((head - tail) <= index)
            return nullptr;
        else
            return &data_buff[(tail + index) & buffer_mask];
    }

    /*!
     * \brief Gets the n'th element on consumed side
     *
     * Unchecked operation, assumes that software already knows if the element can be used, if
     * requested index is out of bounds then reference will point to somewhere inside the buffer
     * The isEmpty() and count() will place appropriate memory barriers if used as loop limiter
     * It is safe to use and modify uint8_t contents only on consumer side
     *
     * \param index Item offset starting on the consumed side
     * \return Reference to requested element, undefined if index exceeds storage count
     */
    uint8_t &operator[](size_t index) {
        return data_buff[(tail + index) & buffer_mask];
    }

    /*!
     * \brief push multiple elements into internal buffer without blocking
     *
     * This function will push as much data as possible from given buffer.
     *
     * \param[in] buff Pointer to buffer with data to be pushed from
     * \param count Number of elements to write from the given buffer
     * \return Number of elements written into internal buffer
     */
    size_t writeBuff(const uint8_t *buff, size_t count);

    /*!
     * \brief push multiple elements into internal buffer without blocking
     *
     * This function will continue writing new entries until all data is written or there is no more space.
     * The callback function can be used to indicate to consumer that it can start fetching data.
     *
     * \warning This function is not deterministic
     *
     * \param[in] buff Pointer to buffer with data to be pushed from
     * \param count Number of elements to write from the given buffer
     * \param count_to_callback Number of elements to write before calling a callback function in first loop
     * \param execute_data_callback Pointer to callback function executed after every loop iteration
     * \return Number of elements written into internal  buffer
     */
    size_t writeBuff(const uint8_t *buff, size_t count, size_t count_to_callback,
                     void (*execute_data_callback)(void));

    /*!
     * \brief Load multiple elements from internal buffer without blocking
     *
     * This function will read up to specified amount of data.
     *
     * \param[out] buff Pointer to buffer where data will be loaded into
     * \param count Number of elements to load into the given buffer
     * \return Number of elements that were read from internal buffer
     */
    size_t readBuff(uint8_t *buff, size_t count);

    /*!
     * \brief Load multiple elements from internal buffer without blocking
     *
     * This function will continue reading new entries until all requested data is read or there is nothing
     * more to read.
     * The callback function can be used to indicate to producer that it can start writing new data.
     *
     * \warning This function is not deterministic
     *
     * \param[out] buff Pointer to buffer where data will be loaded into
     * \param count Number of elements to load into the given buffer
     * \param count_to_callback Number of elements to load before calling a callback function in first iteration
     * \param execute_data_callback Pointer to callback function executed after every loop iteration
     * \return Number of elements that were read from internal buffer
     */
    size_t
    readBuff(uint8_t *buff, size_t count, size_t count_to_callback,
             void (*execute_data_callback)(void));

private:
    constexpr static bool fake_tso = false;
    constexpr static size_t
    buffer_mask = buffer_size - 1;
    size_t head; //!< head index
    size_t tail; //!< tail index

    uint8_t data_buff[buffer_size]; //!< actual buffer
};

#endif // RingBuffer_HPP
