#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QVector>
#include <QtSerialPort/QSerialPort>

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();
    void PlotConfig();
    void readserial();
    void Send(int cmd,uint16_t Info);
    void DoCMD(int cmd, double Data);
    void Draw();
private slots:
    void on_toolButton_clicked();

    void on_toolButton_2_clicked();

    void on_horizontalSlider_valueChanged(int value);

    void on_pushButton_clicked();

private:
    Ui::Widget *ui;

    QSerialPort *ttl;
    QVector<double> RpmGraph;
    QVector<double> CurrentGraph;
    QVector<double> x;
    static const quint16 ttl_vendor_id = 9476;
    static const quint16 ttl_product_id = 768;
    QByteArray serialData;
    QString serialBuffer;
    QString parsed_data;
};
#endif // WIDGET_H


