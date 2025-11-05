#include "main.h"
#include <stdbool.h>

//POOS FUNCTIONS

void drawGUI();
void loading();

static void event_bp();
static void event_bn();
static void event_rp();
static void event_rn();
static void event_s();

static void event_bpr();
static void event_bnr();
static void event_rpr();
static void event_rnr();
static void event_sr();

extern int auton;

extern bool Quals;
extern bool Elims;
extern bool SAWP;

extern int flip;

//Color Sort Variables
extern int ringHueMin;
extern int ringHueMax;
extern int RedMin;
extern int RedMax;
extern int BlueMin;
extern int BlueMax;

