//
// Created by gaoxiang on 2021/1/22.
//
#include <glog/logging.h>
#include <QFileDialog>
#include <utility>

#include "renderGL/gl_api.h"

#include "mapping_interface.h"
#include "qddisplaywidget.h"
#include "quickdebugmainwindow.h"

QDDisplayWidget::QDDisplayWidget(QWidget *parent)
    : QOpenGLWidget(parent), mRubberBand(new QRubberBand(QRubberBand::Rectangle, this)) {
    main_window_ = (QuickDebugMainWindow *)parent;

    mRubberBand->setBackgroundRole(QPalette::Light);
    mRubberBand->setAutoFillBackground(true);
}
QDDisplayWidget::~QDDisplayWidget() = default;

void QDDisplayWidget::SetMappingInterface(std::shared_ptr<mapping::MappingInterface> mapping_interface) {
    map_interface_ = std::move(mapping_interface);
}

void QDDisplayWidget::initializeGL() {
    QOpenGLWidget::initializeGL();
    GLenum err = glewInit();
    if (err != GLEW_OK) {
        LOG(ERROR) << "glew init not good: " << glewGetString(err);
        exit(-1);
    }

    map_interface_->Init();
    map_interface_->Set2DView();

    setMouseTracking(true);
    gl_initialized_ = true;
}

void QDDisplayWidget::resizeGL(int w, int h) {
    if (w != 0 && h != 0) {
        map_interface_->Resize(0, 0, w, h);
        map_interface_->Set2DView();
    }
}

void QDDisplayWidget::paintGL() {
    if (gl_initialized_) {
        map_interface_->Draw();
    }

    QOpenGLWidget::paintGL();
}

void QDDisplayWidget::OnLButtonDown(QPoint point, Qt::KeyboardModifiers modify) {
    down_point_ = point;
    mouse_left_pressed_ = true;
    map_interface_->LButton(point.x(), point.y());
}

void QDDisplayWidget::OnRButtonDown(QPoint point, Qt::KeyboardModifiers modify) {
    IdType picked_id = 0;
    bool found = false;
    auto picked_pos = map_interface_->RButton(point.x(), point.y(), picked_id, found);
    emit valueChanged(found, picked_id, picked_pos);

    mRubberBand->setGeometry(QRect(point, QSize()));
    mRubberBand->show();

    mouse_right_pressed_ = true;
    origin_ = point;
}

void QDDisplayWidget::OnMButtonDown(QPoint point, Qt::KeyboardModifiers modify) {
    this->grabMouse();
    map_interface_->LButton(point.x(), point.y());
}

void QDDisplayWidget::OnLButtonUp(QPoint point, Qt::KeyboardModifiers modify) { mouse_left_pressed_ = false; }

void QDDisplayWidget::OnMButtonUp(QPoint point, Qt::KeyboardModifiers modify) { this->releaseMouse(); }

void QDDisplayWidget::OnRButtonUp(QPoint point, Qt::KeyboardModifiers modify) {
    if (mRubberBand->isVisible()) {
        const QRect zoomRect = mRubberBand->geometry();
        int xp1, yp1, xp2, yp2;
        if (zoomRect.width() > 5 && zoomRect.height() > 5) {
            //获取坐标
            zoomRect.getCoords(&xp1, &yp1, &xp2, &yp2);
            LOG(INFO) << std::abs(xp2 - xp1) << "x" << std::abs(yp2 - yp1) << "+" << xp1 << "+" << yp1;
        }
    }
    mRubberBand->hide();
    this->releaseMouse();
}

void QDDisplayWidget::mousePressEvent(QMouseEvent *event) {
    if (event->button() == Qt::MouseButton::LeftButton) {
        OnLButtonDown(event->pos(), event->modifiers());
    } else if (event->button() == Qt::MouseButton::RightButton) {
        OnRButtonDown(event->pos(), event->modifiers());
    } else if (event->button() == Qt::MouseButton::MiddleButton) {
        OnMButtonDown(event->pos(), event->modifiers());
        mouse_middle_pressed_ = true;
    }
    QWidget::mousePressEvent(event);

    last_global_pos_ = event->pos();
}

void QDDisplayWidget::mouseDoubleClickEvent(QMouseEvent *event) {}

void QDDisplayWidget::mouseMoveEvent(QMouseEvent *event) {
    auto point = event->pos();

    const int dx = event->x() - last_global_pos_.x();
    const int dy = event->y() - last_global_pos_.y();

    if (mouse_middle_pressed_) {
        map_interface_->MouseMove(point.x(), point.y());
        ::V3d p = map_interface_->GetCameraPosition();
        emit ShowCameraPos(p[0], p[1]);

    } else if (mouse_left_pressed_) {
        theta_ -= dx * 0.01f;
        phi_ -= dy * 0.01f;

        phi_ = std::min(M_PI_2 - 0.01, std::max(-M_PI_2 + 0.01, phi_));

        map_interface_->MouseRotate(theta_, phi_);
    } else if (mouse_right_pressed_) {
        mRubberBand->setGeometry(QRect(last_global_pos_, point).normalized());
    }
}

void QDDisplayWidget::mouseReleaseEvent(QMouseEvent *event) {
    if (event->button() == Qt::MouseButton::LeftButton) {
        OnLButtonUp(event->pos(), event->modifiers());
    } else if (event->button() == Qt::MouseButton::MiddleButton) {
        mouse_middle_pressed_ = false;
        OnMButtonUp(event->pos(), event->modifiers());
    } else if (event->button() == Qt::MouseButton::RightButton) {
        mouse_right_pressed_ = false;
        OnRButtonUp(event->pos(), event->modifiers());
    }
}

void QDDisplayWidget::keyPressEvent(QKeyEvent *event) {
    // 没什么需要做的
}

void QDDisplayWidget::keyReleaseEvent(QKeyEvent *event) {}

void QDDisplayWidget::wheelEvent(QWheelEvent *event) {
    if (event->delta() < 0) {
        map_interface_->Zoom(1.1);
    } else {
        map_interface_->Zoom(0.9);
    }
}

bool QDDisplayWidget::OnOpenYAML() {
    LOG(INFO) << "open yaml";

    // 选一个config.yaml
    QFileDialog fd(this, "Select file", "", "yaml配置文件(*.yaml);;");
    if (fd.exec() == QDialog::Accepted) {
        auto filelist = fd.selectedFiles();  //返回文件列表的名称
        if (!map_interface_->ParseYAML(filelist[0].toUtf8().toStdString())) {
            LOG(ERROR) << "Parse YAML failed.";
            return false;
        } else {
            return true;
        }
    } else {
        fd.close();
    }

    return false;
}