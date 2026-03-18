#include "qtStyles.hpp"

QString FOCUS_PID_STYLE = QString(R"(
QDoubleSpinBox {
    background-color: %1;
    color: %2;
    border: 1px solid %3;
    border-radius: 6px;
    padding: 6px 10px;
}
QDoubleSpinBox::up-button,
QDoubleSpinBox::down-button {
    background-color: %3;
    width: 0px;
    border: none;
}
)").arg(ITEM_COLOR).arg(TEXT_COLOR).arg(MAIN_BORDER);

QString NORMAL_PID_STYLE = QString(R"(
QDoubleSpinBox {
    background-color: %1;
    color: %2;
    border: 1px solid %3;
    border-radius: 6px;
    padding: 6px 10px;
    min-height: 24px;
}

/* boutons up/down */
QDoubleSpinBox::up-button,
QDoubleSpinBox::down-button {
    background: %3;        /* même que la bordure */
    width: 0px;
    border: none;
}

/* focus */
QDoubleSpinBox:focus {
    border: 1px solid %4;
}
)").arg(ITEM_BACKGROUND)
.arg(TEXT_COLOR)
.arg(MAIN_BORDER)
.arg(ITEM_COLOR);

QString SLIDER_STYLE = QString(R"(
QSlider::groove:horizontal {
    height: 9px;
    border-radius: 6px;
    background: %1;
}

QSlider::handle:horizontal {
    background: %2;
    border: 2px solid %3;
    width: 9px;
    height: 20px;
    margin: -4px 0;
    border-radius: 7px;
}

QSlider::handle:horizontal:hover {
    background: %4;
}

QSlider::sub-page:horizontal {
    background: %2;
    border-radius: 6px;
    border: 1px solid %5;
}

QSlider::add-page:horizontal {
    background: %6;
    border-radius: 6px;
    border: 1px solid %5;
}
)")
.arg(MAIN_BACKGROUND)   // %1
.arg(ITEM_COLOR)        // %2
.arg(MAIN_BORDER)       // %3
.arg(ACCENT_DANGER)     // %4
.arg(BLACK)             // %5
.arg(ITEM_BACKGROUND);  // %6

QString APP_STYLE = QString(R"(
QWidget {
    background-color: %1;
    color: %2;
    font-size: 13px;
}

/* GROUPBOX */
QGroupBox {
    background-color: %3;
    border: 1px solid %4;
    border-radius: 8px;
    margin-top: 12px;
    padding: 10px;
}

QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 2px 6px;
}

/* LABEL */
QLabel {
    color: %2;
    margin-bottom: 4px;
}

/* SPINBOX */
QDoubleSpinBox {
    background-color: %5;
    color: %2;
    border: 1px solid %4;
    border-radius: 6px;
    padding: 6px 10px;
    min-height: 22px;
}

/* boutons spin */
QDoubleSpinBox::up-button,
QDoubleSpinBox::down-button {
    background-color: %4;
    border: none;
    width: 18px;
}

/* arrows */
QDoubleSpinBox::up-arrow {
    image: none;
    border-left: 5px solid transparent;
    border-right: 5px solid transparent;
    border-bottom: 7px solid %2;
}

QDoubleSpinBox::down-arrow {
    image: none;
    border-left: 5px solid transparent;
    border-right: 5px solid transparent;
    border-top: 7px solid %2;
}

/* hover */
QDoubleSpinBox::up-button:hover,
QDoubleSpinBox::down-button:hover {
    background-color: %6;
}

/* focus */
QDoubleSpinBox:focus {
    border: 1px solid %6;
}

/* SLIDER fallback */
QSlider::groove:horizontal {
    background: %4;
}
)")
.arg(WINDOW_BG)        // %1
.arg(TEXT_COLOR)       // %2
.arg(MAIN_BACKGROUND)  // %3
.arg(MAIN_BORDER)      // %4
.arg(ITEM_BACKGROUND)  // %5
.arg(ITEM_COLOR);      // %6
