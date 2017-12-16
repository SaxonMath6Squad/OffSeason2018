Actions are any objects or things that are meant to be activated or run during the autonomous or user controller.

These can include ball shooters, propellors, arms, etc.

These are meant to be stored in objects that implement the ActionHandler interface and have distinct functions to call

If operations are meant to be threaded, use the start doing action

IMPORTANT:
    Any motors or servos that are run MUST be initialized in our hardware map so that we can kill them appropriately
