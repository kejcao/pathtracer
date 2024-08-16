#include <QApplication>
#include <QImage>
#include <QKeyEvent>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <iostream>
#include <math.h>

import camera;
import scene;
import object;
import math;

const auto s = Scene(parseobj("/home/kjc/untitled.obj"));
// const auto s = Scene(parseobj("examples/cornell-box.obj"));
// const auto s = Scene(parseobj("examples/tea.obj"));

// const auto s = Scene(parseobj("examples/cornell-box.obj"));
auto cam = Camera(vec(0, 0, 0));

class RenderThread : public QThread {
  Q_OBJECT

public:
  RenderThread() {}

signals:
  void updateSignal(const QImage &image);

protected:
  void run() override {
    QImage image(256, 256, QImage::Format_RGB32);
    // cam.render(s, 8, [&, this](int y) {
    //   for (int x = 0; x < 256; ++x) {
    //     const auto c = cam.pixels[y][x];
    //     image.setPixelColor(x, y, QColor(c.x, c.y, c.z));
    //   }
    //   emit updateSignal(image);
    // });
    cam.render(s, 8);
    for (int y = 0; y < 256; ++y) {
      for (int x = 0; x < 256; ++x) {
        const auto c = cam.pixels[y][x];
        image.setPixelColor(x, y, QColor(c.x, c.y, c.z));
      }
    }
    emit updateSignal(image);
  }
};

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow() : QMainWindow() {
    setWindowTitle("Pathtracer GUI");
    setGeometry(100, 100, 800, 600);

    QWidget *centralWidget = new QWidget(this);
    QVBoxLayout *layout = new QVBoxLayout(centralWidget);

    imageLabel = new QLabel(this);
    layout->addWidget(imageLabel);

    setCentralWidget(centralWidget);

    renderThread = new RenderThread();
    connect(renderThread, &RenderThread::updateSignal, this,
            &MainWindow::updateImage_);

    auto movementTimer = new QTimer(this);
    connect(movementTimer, &QTimer::timeout, this, &MainWindow::updateMovement);
    movementTimer->start(16);
  }

  ~MainWindow() { delete renderThread; }

protected:
  void resizeEvent(QResizeEvent *event) override {
    QMainWindow::resizeEvent(event);
    updateImage();
  }

  void keyPressEvent(QKeyEvent *event) override {
    constexpr double speed = .1;
    if (!event->isAutoRepeat()) {
      switch (event->key()) {
      case Qt::Key_W:
        movement.z = speed;
        break;
      case Qt::Key_S:
        movement.z = -speed;
        break;
      case Qt::Key_A:
        movement.x = -speed;
        break;
      case Qt::Key_D:
        movement.x = speed;
        break;
      case Qt::Key_Q:
        movement.y = -speed;
        break;
      case Qt::Key_E:
        movement.y = speed;
        break;
      case Qt::Key_R:
        startRender();
        break;
      default:
        QMainWindow::keyPressEvent(event);
        return;
      }
    }
  }

  void keyReleaseEvent(QKeyEvent *event) override {
    if (!event->isAutoRepeat()) {
      switch (event->key()) {
      case Qt::Key_W:
      case Qt::Key_S:
        movement.z = 0;
        break;
      case Qt::Key_A:
      case Qt::Key_D:
        movement.x = 0;
        break;
      case Qt::Key_Q:
      case Qt::Key_E:
        movement.y = 0;
        break;
      default:
        QMainWindow::keyPressEvent(event);
        return;
      }
    }
  }

  void mousePressEvent(QMouseEvent *event) override {
    lastMousePos = event->pos();
  }

  void mouseMoveEvent(QMouseEvent *event) override {
    if (event->buttons() & Qt::LeftButton) {
      QPoint diff = event->pos() - lastMousePos;
      lastMousePos = event->pos();

      float sensitivity = 0.1f;
      float yaw = diff.x() * sensitivity;

      float pitch = -diff.y() * sensitivity;
      pitch = std::max(-89.0f, std::min(89.0f, pitch)); // no flipping

      cam.direction.y += yaw * M_PI / 180;
      cam.direction.x += pitch * M_PI / 180;
    }
  }

private slots:
  void startRender() { renderThread->start(); }

  void updateMovement() {
    if (movement != vec(0, 0, 0)) {
      cam.origin += movement;
    }
  }

  void updateImage_(const QImage &image) {
    pixmap = QPixmap::fromImage(image);
    updateImage();
  }

  void updateImage() {
    imageLabel->setPixmap(pixmap.scaled(
        imageLabel->width(), imageLabel->height(), Qt::KeepAspectRatio));
  }

private:
  QPixmap pixmap;
  QLabel *imageLabel;
  RenderThread *renderThread;
  QPoint lastMousePos;
  vec movement = vec(0, 0, 0);
};

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  MainWindow window;
  window.show();
  return app.exec();
}

#include "main.moc"
