#include <QApplication>
#include <QImage>
#include <QKeyEvent>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QThread>
#include <QVBoxLayout>
#include <iostream>

import camera;
import scene;
import object;
import math;

const auto s = Scene(parseobj("examples/redcube.obj"));
auto cam = Camera(vec(0, 0, 0));

class RenderThread : public QThread {
  Q_OBJECT

public:
  // RenderThread(Pathtracer* pathtracer) : m_pathtracer(pathtracer) {}

signals:
  void updateSignal(const QImage &image);

protected:
  void run() override {
    QImage image(256, 256, QImage::Format_RGB32);
    cam.render(s, 1);

    for (int y = 0; y < 256; ++y) {
      for (int x = 0; x < 256; ++x) {
        // auto [r,g,b] = cam.pixels[y][x];
        const auto c = cam.pixels[y][x];
        image.setPixelColor(x, y, QColor(c.x, c.y, c.z));
      }
      emit updateSignal(image);
    }
  }

private:
  // Camera* m_pathtracer;
};

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow() : QMainWindow() {
    setWindowTitle("Pathtracer GUI");
    setGeometry(100, 100, 800, 600);

    QWidget *centralWidget = new QWidget(this);
    QVBoxLayout *layout = new QVBoxLayout(centralWidget);

    m_imageLabel = new QLabel(this);
    layout->addWidget(m_imageLabel);

    m_renderButton = new QPushButton("Render", this);
    connect(m_renderButton, &QPushButton::clicked, this,
            &MainWindow::startRender);
    layout->addWidget(m_renderButton);

    setCentralWidget(centralWidget);

    m_renderThread = new RenderThread();
    connect(m_renderThread, &RenderThread::updateSignal, this,
            &MainWindow::updateImage);
  }

  ~MainWindow() { delete m_renderThread; }

protected:
  void keyPressEvent(QKeyEvent *event) override {
    switch (event->key()) {
    case Qt::Key_W:
      cam.origin.x += .2;
      break;
    case Qt::Key_S:
      cam.origin.x -= .2;
      break;
    case Qt::Key_A:
      cam.origin.z += .2;
      break;
    case Qt::Key_D:
      cam.origin.z -= .2;
      break;
    default:
      QMainWindow::keyPressEvent(event);
      return;
    }
    startRender();
  }

private slots:
  void startRender() { m_renderThread->start(); }

  void updateImage(const QImage &image) {
    m_imageLabel->setPixmap(QPixmap::fromImage(image));
  }

private:
  QLabel *m_imageLabel;
  QPushButton *m_renderButton;
  RenderThread *m_renderThread;
};

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  MainWindow window;
  window.show();
  return app.exec();
}

#include "main.moc"
