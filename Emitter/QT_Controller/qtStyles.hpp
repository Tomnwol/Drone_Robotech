#ifndef __QT_STYLE__H__
#define __QT_STYLE__H__

#include <QString>

#define WINDOW_BG        "#242D38"

#define MAIN_BACKGROUND  "#27313F"
#define MAIN_BORDER      "#425266"

#define ITEM_BACKGROUND  "#434c5e"
#define ITEM_BORDER      "#425266"

#define ITEM_COLOR       "#5E85B5"
#define ITEM_HOVER       "#5e81ac"


#define TEXT_COLOR       "#eceff4"
#define SUBTLE_TEXT      "#d8dee9"

#define ACCENT_DANGER    "#ff4d5a"
#define BLACK            "#000000"

#define CLASSIC_CHECKBOX_STYLE "QCheckBox::indicator:unchecked {" "background-color: #ffdddd;" "border: 1px solid gray;" "}" "QCheckBox::indicator:checked {" "background-color: #00AA00;" "border: 1px solid black;" "}"
#define TITLE_LABEL_STYLE "QGroupBox { font-family: 'DejaVu Sans Mono'; font-size: 16px;  font-weight: bold; }"
//#define NORMAL_LABEL_STYLE "QGroupBox { font-weight: normal; }"

#define LOADING_LABEL_STYLE "QCheckBox::indicator:checked { background-color: #555555; border: 1px solid black; }"


extern QString LABEL_CONTROL_STYLE;
extern QString FOCUS_PID_STYLE;
extern QString NORMAL_PID_STYLE;

extern QString SLIDER_STYLE;
extern QString APP_STYLE;


#endif
