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

private:
    Ui::Widget *ui;
    void PlotConfig();
    QSerialPort *ttl;
    QVector<double> RpmGraph;
    QVector<double> x;
    QVector<double> y;
    QVector<double> z;
    QVector<double> w;
    static const quint16 ttl_vendor_id = 9476;
    static const quint16 ttl_product_id = 768;
    QByteArray serialData;
    QString serialBuffer;
    QString parsed_data;
};
#endif // WIDGET_H


