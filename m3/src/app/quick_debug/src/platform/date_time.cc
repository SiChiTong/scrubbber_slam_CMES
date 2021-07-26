//
// Created by gaoxiang on 2019/12/18.
//

#include "platform/date_time.h"
#include <cstdio>
#include <cstring>
#include <ctime>

namespace HAMO {

DateTime::DateTime() {
    // 1990-1-1:00:00:00
    sec_ = 0;
    min_ = 0;
    hour_ = 0;
    day_ = 1;
    mon_ = 1;
    year_ = 1900;
}

DateTime::~DateTime() {}

void DateTime::SetToNow() {
    time_t timep;
    time(&timep);
    struct tm *ptm = localtime(&timep);

    sec_ = ptm->tm_sec;
    min_ = ptm->tm_min;
    hour_ = ptm->tm_hour;
    day_ = ptm->tm_mday;
    mon_ = ptm->tm_mon + 1;
    year_ = ptm->tm_year + 1900;
}

void DateTime::Set(int year, int month, int date) {
    day_ = date;
    mon_ = month;
    year_ = year;
}

void DateTime::Format(char *szBuffer, int bufSize) {
    char temp[64];
    int len = sprintf(temp, "%04d-%02d-%02d %02d:%02d:%02d", year_, mon_, day_,
                      hour_, min_, sec_);
    int size = len < bufSize ? len : (bufSize - 1);
    memcpy(szBuffer, temp, size);
    szBuffer[size] = 0;
}

}  // namespace HAMO
