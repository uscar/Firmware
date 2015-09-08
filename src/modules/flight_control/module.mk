MODULE_COMMAND  = flight_control

ROUTINES = ./routines/altitude_hold.cpp \
           ./routines/landing.cpp \
           ./routines/takeoff.cpp

SRCS    = flight_control_main.cpp \
          routine_controller.cpp

SRCS    += $(ROUTINES)

# Startup handler, the actual app stack size is
# in the task_spawn command
MODULE_STACKSIZE = 3600
