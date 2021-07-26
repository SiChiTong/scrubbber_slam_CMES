#include "mainwindow.h"
#include "logindialog.h"
#include <glog/logging.h>

#include <QApplication>
#include <QStyleFactory>
#include <QtCore/QTextCodec>

int main(int argc, char *argv[]) {
    QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8"));
    QApplication a(argc, argv);
    a.setStyle(QStyleFactory::create("cleanlooks"));

    LoginDialog dlg;
    MappingUIMainWindow w;

    const int max_login_times = 5;
    int try_times = 0;

    while (try_times < max_login_times) {
        if (dlg.exec() == QDialog::Accepted) {
            LOG(INFO) << "login accepted";
            std::string name, token;
            dlg.GetAccountInfo(token, name);
            w.SetAccountInfo(name, token);
            w.setWindowFlags(w.windowFlags() & ~(Qt::WindowMaximizeButtonHint));
            w.setFixedSize(w.width(), w.height());
            w.show();
            return a.exec();
        } else if(dlg.GetFinishStatus() == 0){
            try_times++;
        } else { 
            return 0;
        }
    }

    LOG(ERROR) << "Login failed";
    return 0;

}
