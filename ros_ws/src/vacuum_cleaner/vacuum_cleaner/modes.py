from enum import Enum

class Mode(str, Enum):
    CLEANING = "CLEANING"
    CHARGING = "CHARGING"
    CHARGED = "CHARGED"
    IDLE = "IDLE"  # safe default
