/*
	Copyright 2019  Simon Arlott

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef InterruptSafeBuffer_H_
#define InterruptSafeBuffer_H_

#include <limits.h>

/*!
 * Interrupt-safe buffer of TYPE items. Holds SIZE items (with an extra item in reserve for writing).
 *
 * The item available for reading is retained until it has been read.
 *
 * When there is no space available for writing, if FULL_REUSE_OLDEST is true then the oldest item
 * that has not been read will be overwritten otherwise the current item will be recycled.
 *
 * Data can only be processed as often as dispatch() is called. To process data as quickly as possible,
 * call dispatch() after every push() and (with interrupts disabled) every pop().
 */
template <typename TYPE, int SIZE, bool FULL_REUSE_OLDEST>
class InterruptSafeBuffer {
	static_assert(SIZE > 0, "Buffer cannot be empty");
	static_assert(SIZE < INT_MAX, "Buffer is too large");

private:
	struct Item {
		boolean ready = false;
		struct Item *next;
		TYPE data {};
	};

public:
	InterruptSafeBuffer() {
		for (int i = 0; i < SIZE; i++) {
			items_[i].next = &items_[i + 1];
		}
		tail_->next = nullptr;
	}

	virtual ~InterruptSafeBuffer() = default;

	InterruptSafeBuffer(const InterruptSafeBuffer&) = delete;
	InterruptSafeBuffer& operator=(const InterruptSafeBuffer&) = delete;

	/*!
	 * Check availability of data to be read.
	 *
	 * Safe to call from anywhere.
	 */
	bool inline available() const {
		asm volatile("":::"memory");
		return available_;
	}

	/*!
	 * Get data item to be read. Memory remains valid until pop() is called.
	 *
	 * Safe to call from anywhere.
	 */
	TYPE* read() {
		asm volatile("":::"memory");
		return &read_->data;
	}

	/*!
	 * Release data that has been read.
	 *
	 * Safe to call from anywhere.
	 */
	void pop() {
		read_->ready = false;
		read_->next = nullptr;
		asm volatile("":::"memory");

		available_ = false;
		asm volatile("":::"memory");
	}

	/*!
	 * Get data item to be written. Memory remains valid until push() is called.
	 *
	 * Must only be called from an interrupt handler.
	 */
	TYPE* write() {
		return &write_->data;
	}

	/*!
	 * Mark current data item as available to read.
	 *
	 * Must only be called from an interrupt handler.
	 */
	void push() {
		asm volatile("":::"memory");

		if (!FULL_REUSE_OLDEST) {
			if (write_->next == nullptr) {
				return;
			}
		}

		write_->ready = true;
		write_ = write_->next;

		if (FULL_REUSE_OLDEST) {
			if (write_ == nullptr) {
				write_ = head_;
				head_ = head_->next;

				tail_->next = write_;
				tail_ = write_;
				tail_->next = nullptr;

				write_->ready = false;
			}
		}

		asm volatile("":::"memory");
	}

	/*!
	 * Cleanup after a data item has been read and make the next one available for reading.
	 *
	 * Must be called periodically from an interrupt handler or with interrupts disabled.
	 */
	bool dispatch() {
		bool activity = false;

		asm volatile("":::"memory");

		if (!available_) {
			if (read_ != nullptr) {
				tail_->next = read_;
				tail_ = read_;
				read_ = nullptr;

				activity = true;
			}

			if (head_->ready) {
				read_ = head_;
				head_ = head_->next;
				available_ = true;

				activity = true;
			}
		}

		asm volatile("":::"memory");

		return activity;
	}

private:
	bool available_ = false;
	struct Item *read_ = nullptr;
	struct Item *write_ = &items_[0];
	struct Item *head_ = &items_[0];
	struct Item *tail_ = &items_[SIZE];
	struct Item items_[SIZE + 1] = {};
};

#endif /* InterruptSafeBuffer_H_ */
