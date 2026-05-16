/** \file
 *
 *  Main source file for the Keyboard application. This file contains the main tasks of
 *  the application and is responsible for the initial application hardware configuration.
 */

/**
 * Подключаем PS/2 вход
 * CLOCK = PD0 (INT0) -- D3
 * DATA  = PD1        -- D2
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "Keyboard.h"

#define PS2_CLK_PIN  0
#define PS2_DATA_PIN 1

// static uint8_t break_code = 0;
// static uint8_t current_key = 0;
// static uint8_t modifiers = 0;

static uint8_t modifiers = 0;
static uint8_t keys[6] = {0};

typedef enum
{
	PS2_STATE_NORMAL,
	PS2_STATE_EXTENDED,
	PS2_STATE_BREAK,
	PS2_STATE_EXTENDED_BREAK,
} ps2_state_t;

static ps2_state_t ps2_state = PS2_STATE_NORMAL;
static volatile bool ps2_queue_overflow = false;
static volatile bool ps2_led_pending = false;
static volatile uint8_t ps2_led_state = 0;
static bool report_changed = false;

#define PS2_QUEUE_SIZE 16
#define PS2_QUEUE_MASK (PS2_QUEUE_SIZE - 1)

static volatile uint8_t ps2_queue[PS2_QUEUE_SIZE];
static volatile uint8_t ps2_head = 0;
static volatile uint8_t ps2_tail = 0;

static volatile uint8_t ps2_bitcount = 0;
static volatile uint8_t ps2_byte = 0;

static bool ps2_dequeue(uint8_t *byte)
{
	bool got = false;

	cli();
	if (ps2_tail != ps2_head)
	{
		*byte = ps2_queue[ps2_tail];
		ps2_tail = (ps2_tail + 1) & PS2_QUEUE_MASK;
		got = true;
	}
	sei();

	return got;
}

ISR(INT0_vect)
{
	uint8_t bit = (PIND & (1 << 1)) ? 1 : 0;

	if (ps2_bitcount == 0)
	{
		ps2_bitcount = 1;
		ps2_byte = 0;
		return;
	}

	if (ps2_bitcount >= 1 && ps2_bitcount <= 8)
	{
		ps2_byte >>= 1;
		if (bit)
			ps2_byte |= 0x80;
		ps2_bitcount++;
		return;
	}

	if (ps2_bitcount == 9)
	{
		ps2_bitcount++;
		return;
	}

	if (ps2_bitcount == 10)
	{
		uint8_t next = (ps2_head + 1) & PS2_QUEUE_MASK;

		if (next != ps2_tail)
		{
			ps2_queue[ps2_head] = ps2_byte;
			ps2_head = next;
		}
		else
		{
			ps2_queue_overflow = true;
		}

		ps2_bitcount = 0;
	}
}



/** Buffer to hold the previously generated Keyboard HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevKeyboardHIDReportBuffer[sizeof(USB_KeyboardReport_Data_t)];

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_HID_Device_t Keyboard_HID_Interface =
	{
		.Config =
			{
				.InterfaceNumber              = INTERFACE_ID_Keyboard,
				.ReportINEndpoint             =
					{
						.Address              = KEYBOARD_EPADDR,
						.Size                 = KEYBOARD_EPSIZE,
						.Banks                = 1,
					},
				.PrevReportINBuffer           = PrevKeyboardHIDReportBuffer,
				.PrevReportINBufferSize       = sizeof(PrevKeyboardHIDReportBuffer),
			},
	};

static void add_key(uint8_t key)
{
	for (uint8_t i = 0; i < 6; i++)
	{
		if (keys[i] == key)
			return;

		if (keys[i] == 0)
		{
			keys[i] = key;
			return;
		}
	}
}

static void remove_key(uint8_t key)
{
	for (uint8_t i = 0; i < 6; i++)
	{
		if (keys[i] == key)
			keys[i] = 0;
	}
}

static uint8_t ps2_to_hid(uint8_t code, bool extended);
static void handle_modifier(uint8_t code, uint8_t pressed, bool extended);

static void process_ps2_byte(uint8_t byte)
{
	if (byte == 0xE0 || byte == 0xE1)
	{
		if (ps2_state == PS2_STATE_BREAK)
			ps2_state = PS2_STATE_EXTENDED_BREAK;
		else
			ps2_state = PS2_STATE_EXTENDED;
		return;
	}

	if (byte == 0xF0)
	{
		if (ps2_state == PS2_STATE_EXTENDED)
			ps2_state = PS2_STATE_EXTENDED_BREAK;
		else
			ps2_state = PS2_STATE_BREAK;
		return;
	}

	const bool extended = (ps2_state == PS2_STATE_EXTENDED) ||
	                      (ps2_state == PS2_STATE_EXTENDED_BREAK);
	const bool pressed = (ps2_state == PS2_STATE_NORMAL) ||
	                     (ps2_state == PS2_STATE_EXTENDED);

	handle_modifier(byte, pressed, extended);

	uint8_t key = ps2_to_hid(byte, extended);

	if (key)
	{
		if (pressed)
			add_key(key);
		else
			remove_key(key);

		report_changed = true;
	}

	ps2_state = PS2_STATE_NORMAL;
}

static bool ps2_wait_clock(uint8_t level)
{
	for (uint8_t i = 0; i < 255; i++)
	{
		if (!!(PIND & (1 << PS2_CLK_PIN)) == level)
			return true;

		_delay_us(1);
	}

	return false;
}

static bool ps2_send_byte(uint8_t data)
{
	uint8_t parity = 1;
	bool    ok    = true;

	cli();
	EIMSK &= ~(1 << INT0);

	DDRD |= (1 << PS2_DATA_PIN);
	PORTD &= ~(1 << PS2_DATA_PIN);
	_delay_us(300);

	DDRD &= ~(1 << PS2_CLK_PIN);
	if (!ps2_wait_clock(0))
		ok = false;

	if (ok)
	{
		DDRD |= (1 << PS2_DATA_PIN);
		PORTD &= ~(1 << PS2_DATA_PIN);
		if (!ps2_wait_clock(1) || !ps2_wait_clock(0))
			ok = false;
	}

	for (uint8_t i = 0; ok && i < 8; i++)
	{
		if (data & (1 << i))
		{
			PORTD |= (1 << PS2_DATA_PIN);
			parity++;
		}
		else
		{
			PORTD &= ~(1 << PS2_DATA_PIN);
		}

		if (!ps2_wait_clock(1) || !ps2_wait_clock(0))
			ok = false;
	}

	if (ok)
	{
		if (parity & 1)
			PORTD &= ~(1 << PS2_DATA_PIN);
		else
			PORTD |= (1 << PS2_DATA_PIN);

		if (!ps2_wait_clock(1) || !ps2_wait_clock(0))
			ok = false;
	}

	if (ok)
	{
		PORTD |= (1 << PS2_DATA_PIN);
		if (!ps2_wait_clock(1) || !ps2_wait_clock(0))
			ok = false;
	}

	if (ok)
	{
		DDRD &= ~(1 << PS2_DATA_PIN);
		if (!ps2_wait_clock(1) || !ps2_wait_clock(0))
			ok = false;
	}

	DDRD &= ~((1 << PS2_CLK_PIN) | (1 << PS2_DATA_PIN));
	PORTD |= (1 << PS2_CLK_PIN) | (1 << PS2_DATA_PIN);
	EIMSK |= (1 << INT0);
	sei();

	return ok;
}

static void ps2_set_keyboard_leds(uint8_t leds)
{
	if (ps2_send_byte(0xED))
		ps2_send_byte(leds);
}

static void ps2_flush_pending_leds(void)
{
	uint8_t leds;

	if (!ps2_led_pending)
		return;

	cli();
	leds = ps2_led_state;
	ps2_led_pending = false;
	sei();

	ps2_set_keyboard_leds(leds);
}

static void process_ps2_queue(void)
{
	uint8_t byte;

	if (ps2_queue_overflow)
	{
		cli();
		ps2_queue_overflow = false;
		sei();
		ps2_state = PS2_STATE_NORMAL;
	}

	while (ps2_dequeue(&byte))
		process_ps2_byte(byte);

	ps2_flush_pending_leds();
}

static uint8_t ps2_arrow_to_hid(uint8_t code, bool extended)
{
	if (!extended)
		return 0;

	switch (code)
	{
	case 0x75:
	case 0x48: return HID_KEYBOARD_SC_UP_ARROW;
	case 0x72:
	case 0x50: return HID_KEYBOARD_SC_DOWN_ARROW;
	case 0x6B:
	case 0x4B: return HID_KEYBOARD_SC_LEFT_ARROW;
	case 0x74:
	case 0x4D: return HID_KEYBOARD_SC_RIGHT_ARROW;
	default:   return 0;
	}
}

static uint8_t ps2_keypad_to_hid(uint8_t code)
{
	switch (code)
	{
	case 0x69: return HID_KEYBOARD_SC_KEYPAD_1_AND_END;
	case 0x6B: return HID_KEYBOARD_SC_KEYPAD_4_AND_LEFT_ARROW;
	case 0x72: return HID_KEYBOARD_SC_KEYPAD_2_AND_DOWN_ARROW;
	case 0x73: return HID_KEYBOARD_SC_KEYPAD_5;
	case 0x74: return HID_KEYBOARD_SC_KEYPAD_6_AND_RIGHT_ARROW;
	case 0x75: return HID_KEYBOARD_SC_KEYPAD_8_AND_UP_ARROW;
	case 0x7A: return HID_KEYBOARD_SC_KEYPAD_3_AND_PAGE_DOWN;
	case 0x6C: return HID_KEYBOARD_SC_KEYPAD_7_AND_HOME;
	case 0x7D: return HID_KEYBOARD_SC_KEYPAD_9_AND_PAGE_UP;
	case 0x70: return HID_KEYBOARD_SC_KEYPAD_0_AND_INSERT;
	case 0x71: return HID_KEYBOARD_SC_KEYPAD_DOT_AND_DELETE;
	case 0x4A: return HID_KEYBOARD_SC_KEYPAD_SLASH;
	case 0x7C: return HID_KEYBOARD_SC_KEYPAD_ASTERISK;
	case 0x79: return HID_KEYBOARD_SC_KEYPAD_PLUS;
	default:   return 0;
	}
}

// PS/2 → HID mapping
static uint8_t ps2_to_hid(uint8_t code, bool extended)
{
	uint8_t key = ps2_arrow_to_hid(code, extended);

	if (key)
		return key;

	if (extended)
	{
		switch (code)
		{
		case 0x70: return HID_KEYBOARD_SC_INSERT;
		case 0x71: return HID_KEYBOARD_SC_DELETE;
		case 0x6C: return HID_KEYBOARD_SC_HOME;
		case 0x69: return HID_KEYBOARD_SC_END;
		case 0x7D: return HID_KEYBOARD_SC_PAGE_UP;
		case 0x7A: return HID_KEYBOARD_SC_PAGE_DOWN;
		case 0x1F: return HID_KEYBOARD_SC_LEFT_GUI;
		case 0x7E: return HID_KEYBOARD_SC_SCROLL_LOCK;
		default:   return 0;
		}
	}

	key = ps2_keypad_to_hid(code);

	if (key)
		return key;

	switch (code)
	{
	case 0x1C: return HID_KEYBOARD_SC_A;
	case 0x32: return HID_KEYBOARD_SC_B;
	case 0x21: return HID_KEYBOARD_SC_C;
	case 0x23: return HID_KEYBOARD_SC_D;
	case 0x24: return HID_KEYBOARD_SC_E;
	case 0x2B: return HID_KEYBOARD_SC_F;
	case 0x34: return HID_KEYBOARD_SC_G;
	case 0x33: return HID_KEYBOARD_SC_H;
	case 0x43: return HID_KEYBOARD_SC_I;
	case 0x3B: return HID_KEYBOARD_SC_J;
	case 0x42: return HID_KEYBOARD_SC_K;
	case 0x4B: return HID_KEYBOARD_SC_L;
	case 0x3A: return HID_KEYBOARD_SC_M;
	case 0x31: return HID_KEYBOARD_SC_N;
	case 0x44: return HID_KEYBOARD_SC_O;
	case 0x4D: return HID_KEYBOARD_SC_P;
	case 0x15: return HID_KEYBOARD_SC_Q;
	case 0x2D: return HID_KEYBOARD_SC_R;
	case 0x1B: return HID_KEYBOARD_SC_S;
	case 0x2C: return HID_KEYBOARD_SC_T;
	case 0x3C: return HID_KEYBOARD_SC_U;
	case 0x2A: return HID_KEYBOARD_SC_V;
	case 0x1D: return HID_KEYBOARD_SC_W;
	case 0x22: return HID_KEYBOARD_SC_X;
	case 0x35: return HID_KEYBOARD_SC_Y;
	case 0x1A: return HID_KEYBOARD_SC_Z;

	case 0x45: return HID_KEYBOARD_SC_0_AND_CLOSING_PARENTHESIS;
	case 0x16: return HID_KEYBOARD_SC_1_AND_EXCLAMATION;
	case 0x1E: return HID_KEYBOARD_SC_2_AND_AT;
	case 0x26: return HID_KEYBOARD_SC_3_AND_HASHMARK;
	case 0x25: return HID_KEYBOARD_SC_4_AND_DOLLAR;
	case 0x2E: return HID_KEYBOARD_SC_5_AND_PERCENTAGE;
	case 0x36: return HID_KEYBOARD_SC_6_AND_CARET;
	case 0x3D: return HID_KEYBOARD_SC_7_AND_AMPERSAND;
	case 0x3E: return HID_KEYBOARD_SC_8_AND_ASTERISK;
	case 0x46: return HID_KEYBOARD_SC_9_AND_OPENING_PARENTHESIS;

	case 0x29: return HID_KEYBOARD_SC_SPACE;
	case 0x5A: return HID_KEYBOARD_SC_ENTER;
	case 0x66: return HID_KEYBOARD_SC_BACKSPACE;
	case 0x0D: return HID_KEYBOARD_SC_TAB;

	case 0x4E: return HID_KEYBOARD_SC_MINUS_AND_UNDERSCORE;
	case 0x55: return HID_KEYBOARD_SC_EQUAL_AND_PLUS;

	case 0x54: return HID_KEYBOARD_SC_OPENING_BRACKET_AND_OPENING_BRACE;
	case 0x5B: return HID_KEYBOARD_SC_CLOSING_BRACKET_AND_CLOSING_BRACE;

	case 0x5D: return HID_KEYBOARD_SC_BACKSLASH_AND_PIPE;

	case 0x4C: return HID_KEYBOARD_SC_SEMICOLON_AND_COLON;
	case 0x52: return HID_KEYBOARD_SC_APOSTROPHE_AND_QUOTE;

	case 0x41: return HID_KEYBOARD_SC_COMMA_AND_LESS_THAN_SIGN;
	case 0x49: return HID_KEYBOARD_SC_DOT_AND_GREATER_THAN_SIGN;

	case 0x4A: return HID_KEYBOARD_SC_SLASH_AND_QUESTION_MARK;

	case 0x0E: return HID_KEYBOARD_SC_GRAVE_ACCENT_AND_TILDE;

	case 0x76: return HID_KEYBOARD_SC_ESCAPE;

	case 0x05: return HID_KEYBOARD_SC_F1;
	case 0x06: return HID_KEYBOARD_SC_F2;
	case 0x04: return HID_KEYBOARD_SC_F3;
	case 0x0C: return HID_KEYBOARD_SC_F4;
	case 0x03: return HID_KEYBOARD_SC_F5;
	case 0x0B: return HID_KEYBOARD_SC_F6;
	case 0x83: return HID_KEYBOARD_SC_F7;
	case 0x0A: return HID_KEYBOARD_SC_F8;
	case 0x01: return HID_KEYBOARD_SC_F9;
	case 0x09: return HID_KEYBOARD_SC_F10;
	case 0x78: return HID_KEYBOARD_SC_F11;
	case 0x07: return HID_KEYBOARD_SC_F12;

	case 0x7B: return HID_KEYBOARD_SC_KEYPAD_MINUS;

	case 0x77: return HID_KEYBOARD_SC_NUM_LOCK;
	case 0x58: return HID_KEYBOARD_SC_CAPS_LOCK;

	default: return 0;
	}
}

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	// Init PS/2
	DDRD &= ~((1 << 0) | (1 << 1));
	PORTD |= (1 << 0) | (1 << 1);
	EICRA |= (1 << ISC01);
	//EICRA &= ~(1 << ISC00);
	EIMSK |= (1 << INT0);

	SetupHardware();

	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	GlobalInterruptEnable();

	for (;;)
	{
		process_ps2_queue();
		HID_Device_USBTask(&Keyboard_HID_Interface);
		USB_USBTask();
		ps2_flush_pending_leds();
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware()
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#elif (ARCH == ARCH_XMEGA)
	/* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
#endif

	/* Hardware Initialization */
	Joystick_Init();
	LEDs_Init();
	Buttons_Init();
	USB_Init();
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Keyboard_HID_Interface);

	USB_Device_EnableSOFEvents();

	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	HID_Device_ProcessControlRequest(&Keyboard_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&Keyboard_HID_Interface);
}

static void handle_modifier(uint8_t code, uint8_t pressed, bool extended)
{
	uint8_t mask = 0;

	switch (code)
	{
	case 0x14:
		mask = extended ? HID_KEYBOARD_MODIFIER_RIGHTCTRL : HID_KEYBOARD_MODIFIER_LEFTCTRL;
		break;

	case 0x11:
		mask = extended ? HID_KEYBOARD_MODIFIER_RIGHTALT : HID_KEYBOARD_MODIFIER_LEFTALT;
		break;

	case 0x12:
		mask = HID_KEYBOARD_MODIFIER_LEFTSHIFT;
		break;

	case 0x59:
		mask = HID_KEYBOARD_MODIFIER_RIGHTSHIFT;
		break;

	default:
		return;
	}

	if (pressed)
		modifiers |= mask;
	else
		modifiers &= ~mask;

	report_changed = true;
}

/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
 *  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
 *
 *  \return Boolean \c true to force the sending of the report, \c false to let the library determine if it needs to be sent
 */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                         uint8_t* const ReportID,
                                         const uint8_t ReportType,
                                         void* ReportData,
                                         uint16_t* const ReportSize)
{
	USB_KeyboardReport_Data_t* report = ReportData;

	report->Modifier = modifiers;

	for (uint8_t i = 0; i < 6; i++)
		report->KeyCode[i] = keys[i];

	*ReportSize = sizeof(USB_KeyboardReport_Data_t);

	if (report_changed)
	{
		report_changed = false;
		return true;
	}

	return false;

	// bool dataChanged = false;
	//
	// USB_KeyboardReport_Data_t* report = (USB_KeyboardReport_Data_t*)ReportData;
	//
	// if (ps2_ready)
	// {
	// 	ps2_ready = 0;
	//
	// 	if (ps2_byte == 0xF0)
	// 	{
	// 		break_code = 1;
	// 	}
	// 	else
	// 	{
	// 		uint8_t key = ps2_to_hid(ps2_byte);
	//
	// 		if (key)
	// 		{
	// 			if (break_code)
	// 			{
	// 				break_code = 0;
	// 			}
	// 			else
	// 			{
	// 				report->KeyCode[0] = key;
	// 				dataChanged = true;
	// 			}
	// 		}
	// 	}
	// }
	//
	// *ReportSize = sizeof(USB_KeyboardReport_Data_t);
	// return dataChanged;

	// USB_KeyboardReport_Data_t* KeyboardReport = (USB_KeyboardReport_Data_t*)ReportData;
	//
	// uint8_t JoyStatus_LCL    = Joystick_GetStatus();
	// uint8_t ButtonStatus_LCL = Buttons_GetStatus();
	//
	// uint8_t UsedKeyCodes = 0;
	//
	// if (JoyStatus_LCL & JOY_UP)
	//   KeyboardReport->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_A;
	// else if (JoyStatus_LCL & JOY_DOWN)
	//   KeyboardReport->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_B;
	//
	// if (JoyStatus_LCL & JOY_LEFT)
	//   KeyboardReport->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_C;
	// else if (JoyStatus_LCL & JOY_RIGHT)
	//   KeyboardReport->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_D;
	//
	// if (JoyStatus_LCL & JOY_PRESS)
	//   KeyboardReport->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_E;
	//
	// if (ButtonStatus_LCL & BUTTONS_BUTTON1)
	//   KeyboardReport->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_F;
	//
	// if (UsedKeyCodes)
	//   KeyboardReport->Modifier = HID_KEYBOARD_MODIFIER_LEFTSHIFT;
	//
	// *ReportSize = sizeof(USB_KeyboardReport_Data_t);
	// return false;
}

/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
 *  \param[in] ReportData  Pointer to a buffer where the received report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                          const uint8_t ReportID,
                                          const uint8_t ReportType,
                                          const void* ReportData,
                                          const uint16_t ReportSize)
{
	uint8_t  LEDMask   = LEDS_NO_LEDS;
	uint8_t* LEDReport = (uint8_t*)ReportData;
	uint8_t  ps2Leds   = 0;

	if (*LEDReport & HID_KEYBOARD_LED_NUMLOCK)
	{
		LEDMask |= LEDS_LED1;
		ps2Leds |= 0x02;
	}

	if (*LEDReport & HID_KEYBOARD_LED_CAPSLOCK)
	{
		LEDMask |= LEDS_LED3;
		ps2Leds |= 0x04;
	}

	if (*LEDReport & HID_KEYBOARD_LED_SCROLLLOCK)
	{
		LEDMask |= LEDS_LED4;
		ps2Leds |= 0x01;
	}

	LEDs_SetAllLEDs(LEDMask);

	cli();
	ps2_led_state = ps2Leds;
	ps2_led_pending = true;
	sei();
}

