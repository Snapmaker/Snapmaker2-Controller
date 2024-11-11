/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 * Copyright (c) 2011, 2012 LeafLabs, LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file wirish/HardwareSerial.cpp
 * @brief Wirish serial port implementation.
 */

#include "HardwareSerial.h"

#include <libmaple/libmaple.h>
#include <libmaple/gpio.h>
#include <libmaple/timer.h>
#include <libmaple/usart.h>

#include "../../../../src/hmi/gcode_result_handler.h"

extern "C" {

void __irq_usart1(void) {
    Serial.uart_isr();
}

void __irq_usart2(void) {
    Serial1.uart_isr();
}

void __irq_usart3(void) {
    Serial2.uart_isr();
}

void __irq_uart4(void) {

}

void __irq_uart5(void) {

}

}

// UART3 TX
static void dma_isr_ch2() {
    dma_disable(DMA1, DMA_CH2);
    dma_clear_isr_bits(DMA1, DMA_CH2);
}

// UART3 RX
static void dma_isr_ch3() {
    Serial2.dma_rx_isr();
}

// UART1 TX
static void dma_isr_ch4() {
    dma_disable(DMA1, DMA_CH4);
    dma_clear_isr_bits(DMA1, DMA_CH4);
}

// UART1 RX
static void dma_isr_ch5() {
    Serial.dma_rx_isr();
}

// UART2 RX
static void dma_isr_ch6() {
    Serial1.dma_rx_isr();
}

// UART2 TX
static void dma_isr_ch7() {
    dma_disable(DMA1, DMA_CH7);
    dma_clear_isr_bits(DMA1, DMA_CH7);
}


HardwareSerial::HardwareSerial(usart_dev *usart_device,
                               uint8 tx_pin,
                               uint8 rx_pin) {
    this->usart_device = usart_device;
    this->tx_pin = tx_pin;
    this->rx_pin = rx_pin;
    write_index = 0;
    read_pos = 0;
    write_buff = usart_device->tx_buf;
}

HardwareSerial::HardwareSerial(struct usart_dev *usart_device,
                uint8 tx_pin,
                uint8 rx_pin,
                uint8 n) {
    this->usart_device = usart_device;
    this->tx_pin = tx_pin;
    this->rx_pin = rx_pin;
    this->uart_num = n;
    write_index = 0;
    read_pos = 0;
    write_buff = usart_device->tx_buf;
}

/*
 * Set up/tear down
 */

#if STM32_MCU_SERIES == STM32_SERIES_F1
/* F1 MCUs have no GPIO_AFR[HL], so turn off PWM if there's a conflict
 * on this GPIO bit. */
static void disable_timer_if_necessary(timer_dev *dev, uint8 ch) {
    if (dev != NULL) {
        timer_set_mode(dev, ch, TIMER_DISABLED);
    }
}
#elif (STM32_MCU_SERIES == STM32_SERIES_F2) ||    \
      (STM32_MCU_SERIES == STM32_SERIES_F4)
#define disable_timer_if_necessary(dev, ch) ((void)0)
#else
#warning "Unsupported STM32 series; timer conflicts are possible"
#endif

void HardwareSerial::begin(uint32 baud)
{
	begin(baud,SERIAL_8N1);
}

void HardwareSerial::init_dma()
{
    dma_tube_config dma_cfg;
    usart_reg_map *regs = usart_device->regs;

    dma_cfg.tube_dst_size = DMA_SIZE_8BITS;
    dma_cfg.tube_src_size = DMA_SIZE_8BITS;
    dma_cfg.target_data = 0;

    dma_device = DMA1;
    dma_init(DMA1);

    switch (uart_num) {
    // usart 1
    case 1:
        // TX
        dma_tx_ch = DMA_CH4;
        dma_cfg.tube_req_src = DMA_REQ_SRC_USART1_TX;
        dma_cfg.tube_dst = &regs->DR;
        dma_cfg.tube_src = write_buff;
        dma_cfg.tube_nr_xfers = USART_TX_BUF_SIZE;
        dma_cfg.target_data = 0;
        dma_cfg.tube_flags = DMA_CFG_SRC_INC | DMA_CFG_CMPLT_IE;
        dma_tube_cfg(DMA1, DMA_CH4, &dma_cfg);
        dma_set_priority(DMA1, DMA_CH4, DMA_PRIORITY_MEDIUM);
        nvic_irq_set_priority(DMA1->handlers[DMA_CH4 - 1].irq_line, 7);
        dma_attach_interrupt(DMA1, DMA_CH4, dma_isr_ch4);

        // RX
        dma_rx_ch = DMA_CH5;
        dma_cfg.tube_req_src = DMA_REQ_SRC_USART1_RX;
        dma_cfg.tube_dst = read_buff;
        dma_cfg.tube_src = &regs->DR;
        dma_cfg.tube_nr_xfers = HWSERIAL_RX_BUFFER_SIZE;
        dma_cfg.tube_flags = DMA_CFG_DST_INC | DMA_CFG_CIRC | DMA_CFG_HALF_CMPLT_IE | DMA_CFG_CMPLT_IE;
        dma_tube_cfg(DMA1, DMA_CH5, &dma_cfg);
        dma_set_priority(DMA1, DMA_CH5, DMA_PRIORITY_HIGH);
        nvic_irq_set_priority(DMA1->handlers[DMA_CH5 - 1].irq_line, 4);
        dma_attach_interrupt(DMA1, DMA_CH5, dma_isr_ch5);
        dma_enable(DMA1, DMA_CH5);
        break;

    case 2:
        // TX
        dma_tx_ch = DMA_CH7;
        dma_cfg.tube_req_src = DMA_REQ_SRC_USART2_TX;
        dma_cfg.tube_dst = &regs->DR;
        dma_cfg.tube_src = write_buff;
        dma_cfg.tube_nr_xfers = USART_TX_BUF_SIZE;
        dma_cfg.target_data = 0;
        dma_cfg.tube_flags = DMA_CFG_SRC_INC | DMA_CFG_CMPLT_IE;
        dma_tube_cfg(DMA1, DMA_CH7, &dma_cfg);
        dma_set_priority(DMA1, DMA_CH7, DMA_PRIORITY_MEDIUM);
        nvic_irq_set_priority(DMA1->handlers[DMA_CH7 - 1].irq_line, 6);
        dma_attach_interrupt(DMA1, DMA_CH7, dma_isr_ch7);

        // RX
        dma_rx_ch = DMA_CH6;
        dma_cfg.tube_req_src = DMA_REQ_SRC_USART2_RX;
        dma_cfg.tube_dst = read_buff;
        dma_cfg.tube_src = &regs->DR;
        dma_cfg.tube_nr_xfers = HWSERIAL_RX_BUFFER_SIZE;
        dma_cfg.tube_flags = DMA_CFG_DST_INC | DMA_CFG_CIRC | DMA_CFG_HALF_CMPLT_IE | DMA_CFG_CMPLT_IE;
        dma_tube_cfg(DMA1, DMA_CH6, &dma_cfg);
        dma_set_priority(DMA1, DMA_CH6, DMA_PRIORITY_HIGH);
        nvic_irq_set_priority(DMA1->handlers[DMA_CH6 - 1].irq_line, 3);
        dma_attach_interrupt(DMA1, DMA_CH6, dma_isr_ch6);
        dma_enable(DMA1, DMA_CH6);
        break;

    case 3:
        // TX
        dma_tx_ch = DMA_CH2;
        dma_cfg.tube_req_src = DMA_REQ_SRC_USART3_TX;
        dma_cfg.tube_dst = &regs->DR;
        dma_cfg.tube_src = write_buff;
        dma_cfg.tube_nr_xfers = USART_TX_BUF_SIZE;
        dma_cfg.target_data = 0;
        dma_cfg.tube_flags = DMA_CFG_SRC_INC | DMA_CFG_CMPLT_IE;
        dma_tube_cfg(DMA1, DMA_CH2, &dma_cfg);
        dma_set_priority(DMA1, DMA_CH2, DMA_PRIORITY_MEDIUM);
        nvic_irq_set_priority(DMA1->handlers[DMA_CH2 - 1].irq_line, 8);
        dma_attach_interrupt(DMA1, DMA_CH2, dma_isr_ch2);

        // RX
        dma_rx_ch = DMA_CH3;
        dma_cfg.tube_req_src = DMA_REQ_SRC_USART3_RX;
        dma_cfg.tube_dst = read_buff;
        dma_cfg.tube_src = &regs->DR;
        dma_cfg.tube_nr_xfers = HWSERIAL_RX_BUFFER_SIZE;
        dma_cfg.tube_flags = DMA_CFG_DST_INC | DMA_CFG_CIRC | DMA_CFG_HALF_CMPLT_IE | DMA_CFG_CMPLT_IE;
        dma_tube_cfg(DMA1, DMA_CH3, &dma_cfg);
        dma_set_priority(DMA1, DMA_CH3, DMA_PRIORITY_HIGH);
        nvic_irq_set_priority(DMA1->handlers[DMA_CH3 - 1].irq_line, 5);
        dma_attach_interrupt(DMA1, DMA_CH3, dma_isr_ch3);
        dma_enable(DMA1, DMA_CH3);
        break;

    default:
        return;
    }

    dma_clear_isr_bits(dma_device, dma_tx_ch);
}

void HardwareSerial::check_dma()
{
    if (write_index > 0)
        try_dma_tx();
}

void HardwareSerial::dump_rx_data(uint8_t *buff, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        rb_push_insert(usart_device->rb, buff[i]);
    }
}

void HardwareSerial::rx_process()
{
    uint32_t cur_pos;
    cur_pos = HWSERIAL_RX_BUFFER_SIZE - dma_get_count(dma_device, dma_rx_ch);

    if (cur_pos != read_pos) {
        if (cur_pos > read_pos) {
            dump_rx_data(&read_buff[read_pos], cur_pos - read_pos);
        }
        else {
            dump_rx_data(&read_buff[read_pos], HWSERIAL_RX_BUFFER_SIZE - read_pos);
            if (cur_pos > 0)
                dump_rx_data(&read_buff[0], cur_pos);
        }

        read_pos = cur_pos % HWSERIAL_RX_BUFFER_SIZE;
    }
}

void HardwareSerial::dma_rx_isr()
{
    dma_irq_cause cause = dma_get_irq_cause(dma_device, dma_rx_ch);

    if (cause == DMA_TRANSFER_HALF_COMPLETE || cause == DMA_TRANSFER_COMPLETE) {
        rx_process();
    }
}

void HardwareSerial::uart_isr()
{
    usart_reg_map *regs = usart_device->regs;
    volatile uint32_t tmp;

    if (regs->SR & USART_SR_IDLE) {
        tmp = regs->DR; // to clear IDLE flag
        rx_process();
        tmp = tmp;
    }
}

#include "MapleFreeRTOS1030.h"
bool HardwareSerial::try_dma_tx() {
    bool ret = false;

    taskENTER_CRITICAL();
    // if dma is transferring, cannot switch TX buffer
    if (dma_is_channel_enabled(dma_device, dma_tx_ch)) {
        ret = false;
    }
    else {
        if (0 == write_index) {
            ret = true;
        }
        else {
            dma_disable(dma_device, dma_tx_ch);
            dma_clear_isr_bits(dma_device, dma_tx_ch);

            if (((uint32_t)write_buff) != ((uint32_t)(usart_device->tx_buf))) {
                write_buff = usart_device->tx_buf;
                dma_set_mem_addr(dma_device, dma_tx_ch, bkp_tx_buff);
            }
            else {
                write_buff = bkp_tx_buff;
                dma_set_mem_addr(dma_device, dma_tx_ch, usart_device->tx_buf);
            }
            dma_set_num_transfers(dma_device, dma_tx_ch, write_index);
            write_index = 0;
            dma_enable(dma_device, dma_tx_ch);
            ret = true;
        }
    }
    taskEXIT_CRITICAL();

    return ret;
}

/*
 * Roger Clark.
 * Note. The config parameter is not currently used. This is a work in progress.
 * Code needs to be written to set the config of the hardware serial control register in question.
 *
*/

void HardwareSerial::begin(uint32 baud, uint8_t config)
{
 //   ASSERT(baud <= this->usart_device->max_baud);// Roger Clark. Assert doesn't do anything useful, we may as well save the space in flash and ram etc
    init_dma();

    if (baud > this->usart_device->max_baud) {
        return;
    }

    const stm32_pin_info *txi = &PIN_MAP[this->tx_pin];
    const stm32_pin_info *rxi = &PIN_MAP[this->rx_pin];
    usart_reg_map *regs = usart_device->regs;

    disable_timer_if_necessary(txi->timer_device, txi->timer_channel);

    usart_init(this->usart_device);
    usart_config_gpios_async(this->usart_device,
                             rxi->gpio_device, rxi->gpio_bit,
                             txi->gpio_device, txi->gpio_bit,
                             config);
    usart_set_baud_rate(this->usart_device, USART_USE_PCLK, baud);
    regs->CR3 |= (USART_CR3_DMAT | USART_CR3_DMAR);
    // usart_enable(this->usart_device);

    regs->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE);// don't change the word length etc, and 'or' in the patten not overwrite |USART_CR1_M_8N1);
    regs->CR1 |= USART_CR1_UE;
}

void HardwareSerial::end(void) {
    usart_disable(this->usart_device);
}

/*
 * I/O
 */

int HardwareSerial::read(void) {
	if(usart_data_available(usart_device) > 0) {
		return usart_getc(usart_device);
	} else {
		return -1;
	}
}

int HardwareSerial::available(void) {
    return usart_data_available(this->usart_device);
}

/* Roger Clark. Added function missing from LibMaple code */

int HardwareSerial::peek(void)
{
    return usart_peek(this->usart_device);
}

int HardwareSerial::availableForWrite(void)
{
    return this->usart_device->wb->size-rb_full_count(this->usart_device->wb);
}

size_t HardwareSerial::write(unsigned char ch) {
    gcode_result_handler.GcodeResultCheck(ch);
    return write_directly(ch);
}

size_t HardwareSerial::write_directly(unsigned char ch) {
    taskENTER_CRITICAL();
    if (write_index < USART_TX_BUF_SIZE) {
        write_buff[write_index++] = ch;
        taskEXIT_CRITICAL();
        return 1;
    }
    taskEXIT_CRITICAL();

    while (!try_dma_tx());

    taskENTER_CRITICAL();
    write_buff[write_index++] = ch;
    taskEXIT_CRITICAL();
    return 1;
}

void HardwareSerial::print_directly(const char str[]) {
    if (str == NULL) return;

    uint32_t size = strlen(str);
    uint8_t *ch   = (uint8_t *)str;
    while (size--) {
        write_directly(*ch++);
    }
}

/* edogaldo: Waits for the transmission of outgoing serial data to complete (Arduino 1.0 api specs) */
void HardwareSerial::flush(void) {
    while(!rb_is_empty(this->usart_device->wb)); // wait for TX buffer empty
    while(!((this->usart_device->regs->SR) & (1<<USART_SR_TC_BIT))); // wait for TC (Transmission Complete) flag set
}
