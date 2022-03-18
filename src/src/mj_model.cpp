#include "mj_model.h"

mjModel *m = NULL;
mjData *d = NULL;

std::mutex mtx;

double rtf_des = 1;

double rtf = 0.0;