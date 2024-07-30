#include "ntunepannel.h"
#include "ntunewidget.h"
#include <QStringListModel>
#include <QList>
#include <QHBoxLayout>
#include <QFont>
#include <QDebug>

nTuneMainWidget::nTuneMainWidget(QWidget *parent)
    : QWidget{parent} {

    QList<QString> mainTitles = {
    "General", "SCC", "Torque"
    };

    QList<QList<TuneItemInfo>> mainItems = {
    {
        TuneItemInfo("common.json", "pathFactor", tr("커브에서 너무 밖으로 돌면 올림"),
                     0.96f, 0.9f, 2.0f, 0.01f, 2),
        TuneItemInfo("common.json", "steerActuatorDelay", "",
                     0.2f, 0.1f, 0.5f, 0.05f, 2),
        TuneItemInfo("common.json", "steerRateCost", "",
                     0.45f, 0.2f, 1.5f, 0.05f, 2),
        TuneItemInfo("common.json", "pathOffset", "",
                     0.0f, -1.0f, 1.0f, 0.01f, 2),
    },
    {
        TuneItemInfo("scc.json", "sccGasFactor", tr("scc Gas"),
                     0.98f, 0.5f, 1.5f, 0.01f, 2),
        TuneItemInfo("scc.json", "sccBrakeFactor", tr("scc Brake"),
                     1.0f, 0.5f, 1.5f, 0.01f, 2),
        TuneItemInfo("scc.json", "sccCurvatureFactor", tr("scc Curvature"),
                     0.98f, 0.5f, 1.5f, 0.01f, 2),
    },
    {
        TuneItemInfo("lat_torque_v4.json", "maxLatAccel", "", 3.2f, 1.0f, 4.5f, 0.1f, 1),
        TuneItemInfo("lat_torque_v4.json", "friction", "", 0.01f, 0.0f, 0.2f, 0.01f, 2),
        TuneItemInfo("lat_torque_v4.json", "deadzone", "", 0.0f, 0.0f, 3.0f, 0.01f, 2),
    },
    };

    setStyleSheet(R"(
        * {
          color: white;
          font-size: 50px;
        }
        SettingsWindow {
          background-color: black;
        }
        QStackedWidget, ScrollView, QListView {
          background-color: #292929;
          border-radius: 0px;
        }
        QTabBar::tab { padding: 10px; border-radius: 10px; background-color: transparent; border-bottom: none;}
        QTabBar::tab:selected { background-color: #555555; border-bottom: none;}
        QTabBar::tab:!selected { background-color: transparent; border-bottom: none;}
        QComboBox {
            background-color: transparent;
            border: 2px solid #555555;
            border-radius: 5px;
            font-size: 50px;
        }
        QComboBox QAbstractItemView {
            background: transparent;
            selection-background-color: #555555;
            font-size: 50px;
        }
    )");

    QHBoxLayout *layout = new QHBoxLayout(this);

    QStringList data;
    foreach (auto title, mainTitles) {
        data.append(title);
    }

    QStringListModel *model = new QStringListModel(data);
    listView = new QListView();
    listView->setStyleSheet(R"(
        QListView::item {  padding-top: 30px; padding-bottom: 30px; padding-left: 15px; padding-right: 15px;
            border-radius: 10px; font-weight: bold;}
        QListView::item:selected { background-color: #555555; }
        QListView::item:!selected { background-color: #00000000; }
    )");
    listView->setModel(model);
    BoldItemDelegate *delegate = new BoldItemDelegate(listView);
    listView->setItemDelegate(delegate);
    listView->setSelectionMode(QAbstractItemView::SingleSelection);
    listView->setEditTriggers(QAbstractItemView::NoEditTriggers);
    QItemSelectionModel *selectionModel = listView->selectionModel();
    selectionModel->select(model->index(0, 0), QItemSelectionModel::Select);

    stackedWidget = new QStackedWidget;
    stackedWidget->setStyleSheet("QWidget { background-color: #404040; border-radius: 10px; }");
    foreach (auto items, mainItems) {
        stackedWidget->addWidget(new nTunePannel(items));
    }

    layout->addWidget(listView, 1);
    layout->addWidget(stackedWidget, 4);

    this->setLayout(layout);

    QObject::connect(listView->selectionModel(), &QItemSelectionModel::currentChanged,
                     [&](const QModelIndex &current, const QModelIndex &previous) {
                         qDebug() << "Selected item:" << current.row();
                         this->stackedWidget->setCurrentIndex(current.row());
                     });
}

nTunePannel::nTunePannel(QList<TuneItemInfo>& items, QWidget *parent)
    : QWidget{parent} {

    QVBoxLayout *layout = new QVBoxLayout(this);
    tabBar = new QTabBar;
    tabBar->setStyleSheet(R"(
        QTabBar::tab { font-weight: bold; font-size: 45px; }
    )");
    stackedWidget = new QStackedWidget;
    stackedWidget->setStyleSheet(R"(
        padding: 20px;
    )");

    foreach (const TuneItemInfo item, items) {
        tabBar->addTab(item.key);
        stackedWidget->addWidget(new nTuneWidget(item));
    }

    layout->addWidget(tabBar);
    layout->addWidget(stackedWidget);

    connect(tabBar, &QTabBar::currentChanged, stackedWidget, &QStackedWidget::setCurrentIndex);

    setMouseTracking(true);
}
