#ifndef USB_INTERFACE_H
#define USB_INTERFACE_H

void usb_interface_init(void);
void usb_interface_task(void);
int usb_interface_get_command(char* buffer, int len);
void usb_interface_send_response(const char* response);

#endif // USB_INTERFACE_H
