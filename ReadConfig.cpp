#include"MyHeadFile.h"


/*
********************************************************
函数名：读取配置文件
参数：FName  配置文件名
      cfg    配置信息
函数功能：读取配置信息到项目
返回值：读取配置文件成功则返回true，失败则返回false
********************************************************
*/
bool ReadSATODSConfigInfo(const char FName[], ROVERCFGINFO* cfg) {
    FILE* file = fopen(FName, "r");
    if (file == NULL) {
        printf("Error opening file %s\n", FName);
        return false;
    }

    char line[256];
    while (fgets(line, sizeof(line), file) != NULL) {
        char key[50], value[256];
        if (sscanf(line, "%49[^=]=%255s", key, value) == 2) {
            if (strcmp(key, "IsFileData") == 0) {
                cfg->IsFileData = atoi(value);
            } else if (strcmp(key, "RTKProcMode") == 0) {
                cfg->RTKProcMode = atoi(value);
            } else if (strcmp(key, "BasNetIP") == 0) {
                strncpy(cfg->BasNetIP, value, sizeof(cfg->BasNetIP));
            } else if (strcmp(key, "RovNetIP") == 0) {
                strncpy(cfg->RovNetIP, value, sizeof(cfg->RovNetIP));
            } else if (strcmp(key, "BasNetPort") == 0) {
                cfg->BasNetPort = atoi(value);
            } else if (strcmp(key, "RovNetPort") == 0) {
                cfg->RovNetPort = atoi(value);
            } else if (strcmp(key, "BasObsDatFile") == 0) {
                strncpy(cfg->BasObsDatFile, value, sizeof(cfg->BasObsDatFile));
            } else if (strcmp(key, "RovObsDatFile") == 0) {
                strncpy(cfg->RovObsDatFile, value, sizeof(cfg->RovObsDatFile));
            } else if (strcmp(key, "CodeNoise") == 0) {
                cfg->CodeNoise = atof(value);
            } else if (strcmp(key, "CPNoise") == 0) {
                cfg->CPNoise = atof(value);
            } else if (strcmp(key, "RatioThres") == 0) {
                cfg->RatioThres = atof(value);
            } else if (strcmp(key, "ResFile") == 0) {
                strncpy(cfg->ResFile, value, sizeof(cfg->ResFile));
            }
        }
    }

    fclose(file);
    return true;
}