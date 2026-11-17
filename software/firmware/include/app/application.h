#ifndef APP_APPLICATION_H
#define APP_APPLICATION_H

/* Initialize the timebase and the board's enabled device instances. */
void application_initialize(void);

/* Give each enabled device one opportunity to advance. */
void application_service(void);

#endif
