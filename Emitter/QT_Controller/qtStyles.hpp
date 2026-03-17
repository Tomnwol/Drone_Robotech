#ifndef __QT_STYLE__H__
#define __QT_STYLE__H__

#define CLASSIC_CHECKBOX_STYLE "QCheckBox::indicator:unchecked {" "background-color: #ffdddd;" "border: 1px solid gray;" "}" "QCheckBox::indicator:checked {" "background-color: #00AA00;" "border: 1px solid black;" "}"
#define TITLE_LABEL_STYLE "QGroupBox { font-family: 'DejaVu Sans Mono'; font-size: 16px;  font-weight: bold; }"
#define NORMAL_LABEL_STYLE "QGroupBox { font-weight: normal; }"

#define LOADING_LABEL_STYLE "QCheckBox::indicator:checked { background-color: #555555; border: 1px solid black; }"

#define FOCUS_PID_STYLE "QDoubleSpinBox { background-color: #aaaaff; }"
#define NORMAL_PID_STYLE "QDoubleSpinBox { background-color: #ffffff; }"

#define SLIDER_STYLE R"(
    QSlider::groove:horizontal {
        height: 10px;                  /* épaisseur du slider */
        border-radius: 5px;

    }

    QSlider::handle:horizontal {
        background: #aaaaff;           /* rouge stylé */
        border: 2px solid #444444;
        width: 10px;
        height: 18px;
        margin: -1px 0;                /* centre le handle */
        border-radius: 9px;
    }

    QSlider::handle:horizontal:hover {
        background: #ff4d5a;

    }

    QSlider::sub-page:horizontal {
        background: #aaaaff;           /* partie remplie */
        border-radius: 5px;
        border: 2px solid #000000;
    }

    QSlider::add-page:horizontal {
        background: #4c566a;
        border-radius: 5px;
        border: 2px solid #000000;
    }
    )"
#endif
