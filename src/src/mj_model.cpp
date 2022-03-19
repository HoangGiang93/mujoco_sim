#include "mj_model.h"

mjModel *m = NULL;
mjData *d = NULL;

std::mutex mtx;

double rtf_des = 1;

double rtf = 0.0;

std::experimental::filesystem::path model_path;
std::experimental::filesystem::path tmp_model_path;