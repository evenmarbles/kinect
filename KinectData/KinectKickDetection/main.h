#pragma once

const int width = 640;
const int height = 480;

extern int active_leg;
extern bool record_data;
extern bool save_data;
extern bool guarded_data;
extern bool pickled_data;
extern bool menu_used;

void drawKinectData(void);