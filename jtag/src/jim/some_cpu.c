/*
 * $Id: tap.c $
 *
 * Copyright (C) 2008 Kolja Waschk <kawk>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdio.h>
#include <jim.h>

void some_cpu_tck_rise(jim_device_t *dev, int tms, int tdi)
{
  // jim_print_tap_state(dev);

  switch(dev->tap_state)
  {
    case RESET:
      dev->sreg[0].reg[0] = 0x1; /* IDCODE instruction 0001 */
      dev->sreg[1].reg[0] = 0x87654321; /* Load IDR (fake) */
      dev->current_dr = 1; /* IDR */
      break;

    case UPDATE_IR:
      switch(dev->sreg[0].reg[0])
      {
        case 0x1: /* IDCODE */
          dev->sreg[1].reg[0] = 0x87654321; /* Load IDR (fake) */
          dev->current_dr = 1; /* IDR */
          break;
        case 0xF: /* BYPASS */
        default:
          dev->current_dr = 0; /* BYPASS */
          break;
      }
      break;

    default:
      break;
  }
}

jim_device_t *some_cpu(void)
{
  jim_device_t *dev;
  const int reg_size[2] = { 4 /* IR */, 32 /* IDR */ };

  dev = jim_alloc_device(2, reg_size);

  if(dev)
  {
    dev->tck_rise = some_cpu_tck_rise;
  }

  return dev;
}
