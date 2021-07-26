//
// Created by gaoxiang on 2019/12/18.
//

#ifndef HAMO_LINUX_DATE_TIME_H
#define HAMO_LINUX_DATE_TIME_H

namespace HAMO {

class DateTime {
   public:
    DateTime();

    ~DateTime();

    void SetToNow();

    void Set(int year, int month, int date);

    void Format(char *szBuffer, int bufSize);

   private:
    int sec_;
    int min_;
    int hour_;
    int day_;
    int mon_;
    int year_;
};

}  // namespace HAMO
#endif  // HAMO_LINUX_DATE_TIME_H
