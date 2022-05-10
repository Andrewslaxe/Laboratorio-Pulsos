#include "widget.h"
#include "ui_widget.h"
#include <QVector>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include "stdio.h"
#include "stdlib.h"
#include "cstring"

#define Start 0x06
#define Stop 0x07
double RpmMin = 0, RpmMax = 0, Flag = 0, Rpm = 0, Current = 0, Mediciones = 0;
double siz=0; //Mediciones por size RPMAct por RPm

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    serialBuffer = "";
    parsed_data = "";

    foreach(const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()){
        QString pname = serialPortInfo.portName();
        ui->comboBox->addItem(pname);
    }
    Widget::PlotConfig();
}

Widget::~Widget()
{
   if(ttl->isOpen()){
       ttl->close();
       QObject::disconnect(ttl, SIGNAL(readyRead()), this, SLOT(readSerial()));
   }

    delete ui;
}

void Widget::PlotConfig(){
    ui->myplot->addGraph(ui->myplot->xAxis, ui->myplot->yAxis);
    ui->myplot->addGraph(ui->myplot->xAxis, ui->myplot->yAxis2);
    ui->myplot->graph(0)->setData(x, RpmGraph);
    ui->myplot->graph(1)->setData(x, CurrentGraph);
    ui->myplot->graph(1)->setPen(QPen(Qt::red));
    ui->myplot->graph(0)->setName("RPM");
    ui->myplot->graph(1)->setName("Corriente");
    ui->myplot->plotLayout()->insertRow(0);
    ui->myplot->plotLayout()->addElement(0, 0, new QCPTextElement(ui->myplot, "Velocidad - Motor DC", QFont("sans", 12, QFont::Bold)));
    ui->myplot->legend->setVisible(true);
    QFont legendFont = font();  // start out with MainWindow's font..
    legendFont.setPointSize(8); // and make a bit smaller for legend
    ui->myplot->legend->setFont(legendFont);
    ui->myplot->legend->setBrush(QBrush(QColor(255,255,255,230)));
    ui->myplot->axisRect()->insetLayout()->setInsetAlignment(3, Qt::AlignTop|Qt::AlignRight);
    ui->myplot->xAxis->setLabel("Time Relative");
    ui->myplot->yAxis->setLabel("RPM.");
    ui->myplot->xAxis->setRange(0, 1000);
    ui->myplot->yAxis->setRange(0, 10000);
    ui->myplot->yAxis2->setLabel("mA");
    ui->myplot->yAxis2->setRange(0, 1000);
    ui->myplot->yAxis2->setVisible(true);
    ui->myplot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->myplot->replot();
}

void Widget::Draw(){
    RpmGraph.append(Rpm);
    CurrentGraph.append(Current);
    ui->myplot->graph(0)->setData(x, RpmGraph);
    ui->myplot->graph(1)->setData(x, CurrentGraph);
    ui->myplot->replot();
    ui->myplot->rescaleAxes();
    ui->myplot->update();
    Mediciones++;
    x.append(Mediciones);
}

void Widget::Send(int cmd,uint16_t Info){
    uint8_t Count,Parity=0,Size;
    QByteArray data;
    data.clear();
    data.append(Start);
    if(Info < 256){
        Size = 1;
    }
    else{
        Size = 2;
    }
    data.append(Size);
    data.append(cmd);
    if(Size ==1 ){
        data.append(Info);
    }
    else {
        data.append(0x00FF & (Info >> 8));
        data.append(0x00FF & Info);
    }
    for(Count=0;Count<data.length();Count++){
        Parity^=data.at(Count);
    }
    data.append(Parity);
    data.append(Stop);
    ttl->write(data,data.length());
}
void Widget::readserial()
{
    uint8_t Size,Aux,Parity=0,Check,Temp1,Temp2=0;
    double Data;
    QByteArray buffer ;
    serialData.clear();
    buffer.clear();
    buffer= ttl->readAll();
    serialData.append(buffer);
    ui->label_3->setText("Status: Recibiendo Datos");
    if(serialData.at(0)==Start && serialData.at(serialData.at(1)+4)==Stop){
        Size=serialData.at(1);
        for(Aux=3;Aux<=Size+3;Aux++){
            if(serialData.at(Aux)<0){
                serialData[Aux]=serialData.at(Aux)+256;
            }
        }
        if(Size==1){
            Temp1=(serialData.at(3));
        }
        else{
            Temp1=serialData.at(3)<< 8;
            Temp2=serialData.at(4);
        }

        Data=Temp1|Temp2;
        for(Aux =0;Aux<Size+3;Aux++){
            Parity^=serialData.at(Aux);
        }
        Check=serialData.at(Size+3);
        if(Parity==Check || Parity==Check+256){
            DoCMD(serialData.at(2),Data);
            serialData.clear();
        }
        else{
            qDebug()<< "Se recibió información con errores";
            serialData.clear();
        }
    }
}
void Widget::DoCMD(int cmd, double data){
    QString Datos;
    Datos=QString::number(data);
    qDebug()<< Datos;
    if(cmd==1){     //Se reciben rpm
        Rpm=data;
        if(Flag==0){    //Flag para saber los RPMMin
            RpmMin = Rpm;
            Flag++;
        }
        else{
            if(RpmMin > Rpm){
                RpmMin = Rpm;
            }
            if(RpmMax < Rpm){
                RpmMax = Rpm;
            }
            ui->label_6->setText(QString::number(Rpm));
            ui->label_7->setText(QString::number(RpmMax));
            ui->label_8->setText(QString::number(RpmMin));

        }
    }
    else if(cmd==2){    //Se recibe corriente
        Current = data;
    }

    Draw();

}

void Widget::on_toolButton_clicked()
{
    QString ttl_port_name = ui->comboBox->currentText();
        if(ui->toolButton->text() == "Abrir"){
            foreach(const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()){
                if(ttl_port_name==serialPortInfo.portName()){
                    ttl->setPortName(ttl_port_name);
                    ttl->open(QSerialPort::ReadWrite);
                    ttl->setBaudRate(QSerialPort::Baud115200);
                    ttl->setDataBits(QSerialPort::Data8);
                    ttl->setFlowControl(QSerialPort::NoFlowControl);
                    ttl->setParity(QSerialPort::NoParity);
                    ttl->setStopBits(QSerialPort::OneStop);
                    QObject::connect(ttl, SIGNAL(readyRead()), this, SLOT(readSerial()));
                    Send(1,5);
                    ui->label_3->setText("Status: Conectado");
                    ui->toolButton->setText("Cerrar");

                }
                else{
                    ui->label_3->setText("Puerto no disponible");
                }
            }
        }else{
            Send(0,2); //Le cierra la Flag a la stm para enviar datos
            ttl->close();
            QObject::disconnect(ttl, SIGNAL(readyRead()), this, SLOT(readSerial()));
            ui->toolButton_2->setText("Abrir");
            ui->label_3->setText("Desconectado");
        }
}


void Widget::on_toolButton_2_clicked()
{
    ui->comboBox->clear();
        foreach(const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()){
            QString pname = serialPortInfo.portName();
            ui->comboBox->addItem(pname);
        }
}


void Widget::on_horizontalSlider_valueChanged(int value)
{
    QString Slide="Valor Ingresado: "+QString::number(value);
    ui->label_9->setText(Slide);
    Send(3,value);
}


void Widget::on_pushButton_clicked()
{
    QString Txt1, Txt2;
    uint16_t Pwm,Time;
            Txt1 = ui->lineEdit->text();
            Txt2 = ui->lineEdit_2->text();
            if(ui->lineEdit->text() != "" && ui->lineEdit_2->text() != ""){
                if(Txt1.toInt()>=0 && Txt1.toInt()<100){
                    if(Txt2.toInt()>=0){
                        Txt2 = ui->lineEdit_2->text();
                        Pwm=Txt1.toInt();
                        Time=Txt2.toInt();
                        Send(4,Time);  //Va hasta 100
                        while(ttl->waitForBytesWritten()==0);
                        Send(2,Pwm);
                    }
                    else{
                        QMessageBox msgBox;
                        msgBox.setText("Ingrese un valor de tiempo valido");
                        msgBox.exec();
                    }
                }
                else{
                    QMessageBox msgBox;
                    msgBox.setText("En PWM ingrese un valor de 0 a 100");
                    msgBox.exec();
                }

            }else{
            QMessageBox msgBox;
            msgBox.setText("Escriba un numero en el campo del PWM y Tiempo");
            msgBox.exec();
    }
}

