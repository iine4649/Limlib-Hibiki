#include "liblvgl/core/lv_disp.h"
#include "liblvgl/core/lv_group.h"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/core/lv_obj_style.h"
#include "liblvgl/font/lv_font.h"
#include "liblvgl/misc/lv_area.h"
#include "liblvgl/misc/lv_color.h"
#include "liblvgl/misc/lv_style.h"
#include "liblvgl/widgets/lv_canvas.h"
#include "liblvgl/widgets/lv_img.h"
#include "liblvgl/widgets/lv_label.h"
#include "main.h"
#include "liblvgl/lvgl.h"
#include "pros/poos.hpp"
//#include "liblvgl/New_Project_1.h"


lv_group_t * bluePos;
lv_group_t * blueNeg;
lv_group_t * redPos;
lv_group_t * redNeg;
lv_group_t * awpRed;
lv_group_t * awpBlue;
lv_group_t * cornerSw;
lv_group_t * autonInfo;

lv_event_cb_t  bluePosClick;
lv_event_cb_t * blueNegClick;
lv_event_cb_t * redPosClick;
lv_event_cb_t * redNegClick;
lv_event_cb_t * awpRedClick;
lv_event_cb_t * awpBlueClick;

lv_event_cb_t  bluePosReleased;
lv_event_cb_t * blueNegReleased;
lv_event_cb_t * redPosReleased;
lv_event_cb_t * redNegReleased;
lv_event_cb_t * awpRedReleased;
lv_event_cb_t * awpBlueReleased;


lv_obj_t * bluePosBtn;
lv_obj_t * bluePosLabel;
lv_obj_t * blueNegBtn;
lv_obj_t * blueNegLabel;
lv_obj_t * redPosBtn;
lv_obj_t * redPosLabel;
lv_obj_t * redNegBtn;
lv_obj_t * redNegLabel;
lv_obj_t * awpRedBtn;
lv_obj_t * awpRedLabel;

lv_obj_t * myButtonLabel;
lv_obj_t * allianceSwitch;
lv_obj_t * cornerSwitch;
lv_obj_t * obj;
//lv_obj_t * autonPoints;
//lv_obj_t * autonTime;
lv_obj_t * autonBox;
lv_obj_t * colorSort;
//lv_obj_t * autonTimeEdit;
//lv_obj_t * autonPointsEdit;
lv_obj_t * colorSortEdit;
lv_obj_t * autonName;
lv_obj_t * robotSetup;
lv_obj_t * robotSetupEdit;
//lv_obj_t * robotPath;
//lv_obj_t * robotPathEdit;
lv_obj_t * hibikiVerLabel;
lv_obj_t * spinner;
lv_obj_t * logo = lv_img_create(lv_scr_act());


//Used to determine what Auton is selected by rotating the number
int auton = 0;

//Used to switch which side the auton is on to fix the asymmetrical field layout


//Elims and Quals and SAWP Bools
bool Quals = false;
bool Elims = false;
bool SAWP = false;

//Color Sort Variables
int ringHueMin;
int ringHueMax;
int BlueMin = 180;
int BlueMax = 225;
int RedMin = 0;
int RedMax = 40;

static void event_rc(lv_event_t * redChecked) {

}
// Blue Positive Event
static void event_bp(lv_event_t * bluePosClick) {
    lv_label_set_text(autonName, "Blue Positive\nMogo Rush");
    //lv_label_set_text(autonTimeEdit, "13 Seconds");
    //lv_label_set_text(autonPointsEdit, "x1 Alliance Stake = 3\nx4 Mogo = 3\n\nTotal = 6 + AWP");
    lv_label_set_text(colorSortEdit, "WIP");
    //lv_label_set_text(robotPathEdit, "WIP");
    lv_label_set_text(robotSetupEdit, "WIP");
    auton = 1;

    //Switch Ring Sort Colour To Red
    ringHueMin = RedMin;
    ringHueMax = RedMax;
}

static void event_bpr(lv_event_t * bluePosReleased) {
    lv_label_set_text(bluePosLabel, "Blue\nMogo\nRush");
}

static void event_bn(lv_event_t * blueNegClick) {
    lv_label_set_text(autonName, "Blue Negative\nRing Stack");
    //lv_label_set_text(autonTimeEdit, "13 Seconds");
    //lv_label_set_text(autonPointsEdit, "x1 Alliance Stake = 3\nx4 Mogo = 3\n\nTotal = 6 + AWP");
    lv_label_set_text(colorSortEdit, "Sorting Red Rings");
    //lv_label_set_text(robotPathEdit, "1) Score 1 ring on allinace stake\n2) Pick up mobile goal\n3) Pick up ring\n4) Clear corner\n5) Touch Tower");
    lv_label_set_text(robotSetupEdit, "Place robot between tile\n2&3 from the right side,\nmogo mech facing outward,\nmogo mech bar on the\ngap between tiles");
    auton = 3;

    //Switch Ring Sort Colour To Red
    ringHueMin = RedMin;
    ringHueMax = RedMax;
}

static void event_bnr(lv_event_t * blueNegReleased) {
    lv_label_set_text(blueNegLabel, "Blue\nRing\nStack");
}

static void event_rp(lv_event_t * redPosClick) {
    lv_label_set_text(autonName, "Red Positive\nMogo Rush");
    //lv_label_set_text(autonTimeEdit, "13 Seconds");
    //lv_label_set_text(autonPointsEdit, "x1 Alliance Stake = 3\nx4 Mogo = 3\n\nTotal = 6 + AWP");
    lv_label_set_text(colorSortEdit, "Sorting Blue Rings");
    //lv_label_set_text(robotPathEdit, "1) Score 1 ring on allinace stake\n2) Pick up mobile goal\n3) Pick up ring\n4) Clear corner\n5) Touch Tower");
    lv_label_set_text(robotSetupEdit, "Place robot between tile\n2&3 from the right side,\nmogo mech facing outward,\nmogo mech bar on the\ngap between tiles");
    auton = 1;

    //Switch Ring Sort Colour To Blue
    ringHueMin = BlueMin;
    ringHueMax = BlueMax;
}

static void event_rpr(lv_event_t * redPosReleased) {
    lv_label_set_text(redPosLabel, "Red\nMogo\nRush");
}

static void event_rn(lv_event_t * redNegClick) {
    lv_label_set_text(autonName, "Red Negative\nRing Stack");
    //lv_label_set_text(autonTimeEdit, "13 Seconds");
    //lv_label_set_text(autonPointsEdit, "x1 Alliance Stake = 3\nx1 Mogo = 3\n\nTotal = 6 + AWP");
    lv_label_set_text(colorSortEdit, "Sorting Blue Rings");
    //lv_label_set_text(robotPathEdit, "1) Score 1 ring on allinace stake\n2) Pick up mobile goal\n3) Pick up ring\n4) Clear corner\n5) Touch Tower");
    lv_label_set_text(robotSetupEdit, "Place robot between tile\n2&3 from the left side,\nmogo mech facing outward,\nmogo mech bar on the\ngap between tiles 1&2 from you");
    auton = 3;

    //Switch Ring Sort Colour To Blue
    ringHueMin = BlueMin;
    ringHueMax = BlueMax;
}

static void event_rnr(lv_event_t * redNegReleased) {
    lv_label_set_text(redNegLabel, "Red\nRing\nStack");
}

static void event_s(lv_event_t * awpRedClick) {
    lv_label_set_text(autonName, "AWP Red Auton");
    //lv_label_set_text(autonTimeEdit, "60 Seconds");
    //lv_label_set_text(autonPointsEdit, "x2 Alliance Stake = 6\nx2 5 Ring Mogo = 16\nx4 Corner = 20\n\nTotal = 40");
    lv_label_set_text(colorSortEdit, "Sorting Blue Rings");
    //lv_label_set_text(robotPathEdit, "1) Score Alliance Stake\n2) Fill Right Goal\n3) Fill Left Goal\n4) Back Goals\n5) Back Alliance Stake");
    lv_label_set_text(robotSetupEdit, "Place robot between tile\n3&4, have pre-rollers face\noutward, pre-rollers on gap\nbetween tiles 1&2\nfrom you");
    auton = 2;

    //Switch Ring Sort Colour To Blue
    ringHueMin = BlueMin;
    ringHueMax = BlueMax;
}

static void event_sr(lv_event_t * awpRedReleased) {
    lv_label_set_text(awpRedLabel, "AWP\nRed\nAuton");
}


//POOS FUNCTIONS
void drawGUI() {
  
    //Not Pressed Blue Style
    static lv_style_t styleBlue;
    lv_style_init(&styleBlue);

    lv_style_set_radius(&styleBlue, 1);

    lv_style_set_bg_opa(&styleBlue, LV_OPA_100);
    lv_style_set_bg_color(&styleBlue, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_bg_grad_color(&styleBlue, lv_palette_darken(LV_PALETTE_BLUE, 2));
    lv_style_set_bg_grad_dir(&styleBlue, LV_GRAD_DIR_VER);

    lv_style_set_border_opa(&styleBlue, LV_OPA_40);
    lv_style_set_border_width(&styleBlue, 2);
    lv_style_set_border_color(&styleBlue, lv_palette_main(LV_PALETTE_GREY));

    lv_style_set_shadow_width(&styleBlue, 3);
    lv_style_set_shadow_color(&styleBlue, lv_palette_main(LV_PALETTE_GREY));
    lv_style_set_shadow_ofs_y(&styleBlue, 8);

    lv_style_set_outline_opa(&styleBlue, LV_OPA_COVER);
    lv_style_set_outline_color(&styleBlue, lv_palette_main(LV_PALETTE_BLUE));

    lv_style_set_text_color(&styleBlue, lv_color_white());
    lv_style_set_pad_all(&styleBlue, 10);

    /*
    LV_IMG_DECLARE(New_Project_1);
    lv_img_set_src(logo, &New_Project_1);
    lv_obj_center(logo);*/


    //pressed blue Style
    static lv_style_t styleBlue_pr;
    lv_style_init(&styleBlue_pr);

    /*Add a large outline when pressed*/
    lv_style_set_outline_width(&styleBlue_pr, 30);
    lv_style_set_outline_opa(&styleBlue_pr, LV_OPA_TRANSP);

    lv_style_set_translate_y(&styleBlue_pr, 5);
    lv_style_set_shadow_ofs_y(&styleBlue_pr, 3);
    lv_style_set_bg_color(&styleBlue_pr, lv_palette_darken(LV_PALETTE_BLUE, 2));
    lv_style_set_bg_grad_color(&styleBlue_pr, lv_palette_darken(LV_PALETTE_BLUE, 4));

    //Not Pressed Red Style
    static lv_style_t styleRed;
    lv_style_init(&styleRed);

    lv_style_set_radius(&styleRed, 1);

    lv_style_set_bg_opa(&styleRed, LV_OPA_100);
    lv_style_set_bg_color(&styleRed, lv_palette_main(LV_PALETTE_RED));
    lv_style_set_bg_grad_color(&styleRed, lv_palette_darken(LV_PALETTE_RED, 2));
    lv_style_set_bg_grad_dir(&styleRed, LV_GRAD_DIR_VER);

    lv_style_set_border_opa(&styleRed, LV_OPA_40);
    lv_style_set_border_width(&styleRed, 2);
    lv_style_set_border_color(&styleRed, lv_palette_main(LV_PALETTE_GREY));

    lv_style_set_shadow_width(&styleRed, 3);
    lv_style_set_shadow_color(&styleRed, lv_palette_main(LV_PALETTE_GREY));
    lv_style_set_shadow_ofs_y(&styleRed, 8);

    lv_style_set_outline_opa(&styleRed, LV_OPA_COVER);
    lv_style_set_outline_color(&styleRed, lv_palette_main(LV_PALETTE_RED));

    lv_style_set_text_color(&styleRed, lv_color_white());
    lv_style_set_pad_all(&styleRed, 10);

    //pressed Red Style
    static lv_style_t styleRed_pr;
    lv_style_init(&styleRed_pr);

    /*Add a large outline when pressed*/
    lv_style_set_outline_width(&styleRed_pr, 30);
    lv_style_set_outline_opa(&styleRed_pr, LV_OPA_TRANSP);

    lv_style_set_translate_y(&styleRed_pr, 5);
    lv_style_set_shadow_ofs_y(&styleRed_pr, 3);
    lv_style_set_bg_color(&styleRed_pr, lv_palette_darken(LV_PALETTE_RED, 2));
    lv_style_set_bg_grad_color(&styleRed_pr, lv_palette_darken(LV_PALETTE_RED, 4));

    static lv_style_t redToggle;
    lv_style_init(&redToggle);

    lv_style_set_bg_color(&redToggle, lv_palette_main(LV_PALETTE_RED));
    lv_style_set_bg_grad_color(&redToggle, lv_palette_darken(LV_PALETTE_RED, 2));

    static lv_style_t blueToggle;
    lv_style_init(&blueToggle);

    lv_style_set_bg_color(&blueToggle, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_bg_grad_color(&blueToggle, lv_palette_darken(LV_PALETTE_BLUE, 2));
    
    static lv_style_t posToggle;
    lv_style_init(&posToggle);

    lv_style_set_bg_color(&posToggle, lv_palette_main(LV_PALETTE_BLUE_GREY));
    lv_style_set_bg_grad_color(&posToggle, lv_palette_darken(LV_PALETTE_BLUE_GREY, 2));

    static lv_style_t negToggle;
    lv_style_init(&negToggle);

    lv_style_set_bg_color(&negToggle, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_bg_grad_color(&negToggle, lv_palette_darken(LV_PALETTE_BLUE, 2));

    //Auton Box Style
    static lv_style_t boxStyle;
    lv_style_init(&boxStyle);

    lv_style_set_bg_color(&boxStyle, lv_palette_darken(LV_PALETTE_GREY, 4));
    lv_style_set_border_color(&boxStyle, lv_color_white());
    
    //Big Text
    static lv_style_t bigText;
    lv_style_init(&bigText);

    lv_style_set_text_font(&bigText, &lv_font_montserrat_20);

    //Creating Groups
    bluePos = lv_group_create();
    blueNeg = lv_group_create();
    redPos = lv_group_create();
    redNeg = lv_group_create();
    awpRed = lv_group_create();
    //cornerSw = lv_group_create();
    autonInfo = lv_group_create();

    //Creating Objects
    //allianceSwitch = lv_switch_create(lv_scr_act());
    //cornerSwitch = lv_switch_create(lv_scr_act());
    autonBox = lv_obj_create(lv_scr_act());
    //autonTime = lv_label_create(autonBox);
    //autonPoints = lv_label_create(autonBox);
    colorSort = lv_label_create(autonBox);
    autonName = lv_label_create(autonBox);
    //robotPath = lv_label_create(autonBox);
    robotSetup = lv_label_create(autonBox);
    //autonPointsEdit = lv_label_create(autonBox);
    //autonTimeEdit = lv_label_create(autonBox);
    colorSortEdit = lv_label_create(autonBox);
    //robotPathEdit = lv_label_create(autonBox);
    robotSetupEdit = lv_label_create(autonBox);

    hibikiVerLabel = lv_label_create(lv_scr_act());

    bluePosBtn = lv_btn_create(lv_scr_act());
    bluePosLabel = lv_label_create(bluePosBtn);
    blueNegBtn = lv_btn_create(lv_scr_act());
    blueNegLabel = lv_label_create(blueNegBtn);
    redPosBtn = lv_btn_create(lv_scr_act());
    redPosLabel = lv_label_create(redPosBtn);
    redNegBtn = lv_btn_create(lv_scr_act());
    redNegLabel = lv_label_create(redNegBtn);
    awpRedBtn = lv_btn_create(lv_scr_act());
    awpRedLabel = lv_label_create(awpRedBtn);


    //Assigning Objects into Groups
    lv_group_add_obj(bluePos, bluePosBtn);
    lv_group_add_obj(blueNeg, blueNegBtn);
    lv_group_add_obj(redPos, redPosBtn);
    lv_group_add_obj(redNeg, redNegBtn);
    lv_group_add_obj(awpRed, awpRedBtn);

    lv_group_add_obj(bluePos, bluePosLabel);
    lv_group_add_obj(blueNeg, blueNegLabel);
    lv_group_add_obj(redPos, redPosLabel);
    lv_group_add_obj(redNeg, redPosLabel);
    lv_group_add_obj(awpRed, awpRedLabel);

    //lv_group_add_obj(cornerSw, cornerSwitch);
    lv_group_add_obj(autonInfo, autonBox);
    //lv_group_add_obj(autonInfo, autonTime);
    //lv_group_add_obj(autonInfo, autonPoints);
    lv_group_add_obj(autonInfo, colorSort);
    //lv_group_add_obj(autonInfo, robotPath);
    lv_group_add_obj(autonInfo, robotSetup);
    //lv_group_add_obj(autonInfo, autonTimeEdit);
    //lv_group_add_obj(autonInfo, autonPointsEdit);
    lv_group_add_obj(autonInfo, colorSortEdit);
    //lv_group_add_obj(autonInfo, robotPathEdit);
    lv_group_add_obj(autonInfo, robotSetupEdit);



    //Creating Blue Positive Button
    lv_label_set_text(bluePosLabel, "Blue\nMogo\nRush");
    lv_obj_center(bluePosLabel);
    lv_obj_add_style(bluePosBtn, &styleBlue, 0);
    lv_obj_add_style(bluePosBtn, &styleBlue_pr, LV_STATE_PRESSED);
    lv_obj_set_size(bluePosBtn, 70, 70);
    lv_obj_align(bluePosBtn, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_add_event_cb(bluePosBtn, event_bp, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(bluePosBtn, event_bpr, LV_EVENT_RELEASED, NULL);

    lv_label_set_text(blueNegLabel, "Blue\nRing\nStack");
    lv_obj_center(blueNegLabel);
    lv_obj_add_style(blueNegBtn, &styleBlue, 0);
    lv_obj_add_style(blueNegBtn, &styleBlue_pr, LV_STATE_PRESSED);
    lv_obj_set_size(blueNegBtn, 70, 70);
    lv_obj_align(blueNegBtn, LV_ALIGN_LEFT_MID, 10, 10);
    lv_obj_add_event_cb(blueNegBtn, event_bn, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(blueNegBtn, event_bnr, LV_EVENT_RELEASED, NULL);

    lv_label_set_text(redPosLabel, "Red\nMogo\nRush");
    lv_obj_center(redPosLabel);
    lv_obj_add_style(redPosBtn, &styleRed, 0);
    lv_obj_add_style(redPosBtn, &styleRed_pr, LV_STATE_PRESSED);
    lv_obj_set_size(redPosBtn, 70, 70);
    lv_obj_align(redPosBtn, LV_ALIGN_TOP_LEFT, 90, 10);
    lv_obj_add_event_cb(redPosBtn, event_rp, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(redPosBtn, event_rpr, LV_EVENT_RELEASED, NULL);

    lv_label_set_text(redNegLabel, "Red\nRing\nStack");
    lv_obj_center(redNegLabel);
    lv_obj_add_style(redNegBtn, &styleRed, 0);
    lv_obj_add_style(redNegBtn, &styleRed_pr, LV_STATE_PRESSED);
    lv_obj_set_size(redNegBtn, 70, 70);
    lv_obj_align(redNegBtn, LV_ALIGN_LEFT_MID, 90, 10);
    lv_obj_add_event_cb(redNegBtn, event_rn, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(redNegBtn, event_rnr, LV_EVENT_RELEASED, NULL);

    lv_label_set_text(awpRedLabel, "Red\nAWP\nAuton");
    lv_obj_center(awpRedLabel);
    lv_obj_add_style(awpRedBtn, &styleRed, 0);
    lv_obj_add_style(awpRedBtn, &styleRed_pr, LV_STATE_PRESSED);
    lv_obj_set_size(awpRedBtn, 70, 70);
    lv_obj_align(awpRedBtn, LV_ALIGN_TOP_LEFT, 170, 10);
    lv_obj_add_event_cb(awpRedBtn, event_s, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(awpRedBtn, event_sr, LV_EVENT_RELEASED, NULL);

    /*lv_obj_add_style(allianceSwitch, &redToggle, LV_STATE_DEFAULT);
    lv_obj_add_style(allianceSwitch, &blueToggle, LV_STATE_CHECKED);
    lv_obj_set_size(allianceSwitch, 100, 50);
    lv_obj_align(allianceSwitch, LV_ALIGN_BOTTOM_LEFT, 20, -10);
    
    lv_obj_add_style(cornerSwitch, &posToggle, LV_STATE_DEFAULT);
    lv_obj_add_style(cornerSwitch, &negToggle, LV_STATE_CHECKED);
    lv_obj_set_size(cornerSwitch, 100, 50);
    lv_obj_align(cornerSwitch, LV_ALIGN_BOTTOM_LEFT, 130, -10);*/
    
    //Text To Display Poos Version
    lv_label_set_text(hibikiVerLabel, "INAZUMA V2.1");
    lv_obj_add_style(hibikiVerLabel, &bigText, 0);
    lv_obj_align(hibikiVerLabel, LV_ALIGN_BOTTOM_LEFT, 70, -25);

    //Create Box To Show Auton Info
    
    lv_obj_add_style(autonBox, &boxStyle, LV_STATE_DEFAULT);
    lv_obj_set_size(autonBox, 230, 230);
    lv_obj_align(autonBox, LV_ALIGN_CENTER, 120, 0);
    
    //Create Time Display For Autons
    //lv_obj_align(autonTime, LV_ALIGN_CENTER, 0, -50);
    //lv_obj_align(autonTimeEdit, LV_ALIGN_CENTER, 0, 0);
    //lv_obj_add_style(autonTimeEdit, &bigText, 0);
    //lv_label_set_text(autonTime, "Time To Complete Auton:");

    //Create Points Breakdown For Autons
    //lv_obj_align(autonPoints, LV_ALIGN_CENTER, 0, 80);
    //lv_obj_align(autonPointsEdit, LV_ALIGN_CENTER, 0, 150);
    //lv_obj_add_style(autonPointsEdit, &bigText, 0);
    //lv_label_set_text(autonPoints, "Points Break Down:");

    //Create Colour Sort Info For Autons
    lv_obj_align(colorSort, LV_ALIGN_CENTER, 0, -50);
    lv_obj_align(colorSortEdit, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_style(colorSortEdit, &bigText, 0);
    lv_label_set_text(colorSort, "Colour Sorting:");

    //Create Name For The Auton Info
    lv_obj_align(autonName, LV_ALIGN_CENTER, 0, -90);
    lv_obj_add_style(autonName, &bigText, 0);
    lv_label_set_text(autonName, "Auton Name:");
    
    //Create Robot Path For Autons
    //lv_obj_align(robotPath, LV_ALIGN_CENTER, 0, 300);
    //lv_obj_align(robotPathEdit, LV_ALIGN_CENTER, 0, 360);
    //lv_label_set_text(robotPath, "Robot Path:");

    //Create Robot Setup For Autons
    lv_obj_align(robotSetup, LV_ALIGN_CENTER, 0, -70);
    lv_obj_align(robotSetupEdit, LV_ALIGN_CENTER, 0, -100);
    lv_label_set_text(robotSetup, "Where To Set Up Robot:");
    
}

void loading() {
    spinner = lv_spinner_create(lv_scr_act(), 1500, 50);
    lv_obj_set_size(spinner, 60, 60);
    lv_obj_align(spinner, LV_ALIGN_CENTER, 0, 0);
    lv_obj_del_delayed(spinner, 2000);
}