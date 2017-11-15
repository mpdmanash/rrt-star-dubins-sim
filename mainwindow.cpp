#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    renderArea = ui->renderArea;
    rrtstar = renderArea->rrtstar;
    simulated = false;
}

/**
 * @brief Start the simulator.
 */
void MainWindow::on_startButton_clicked()
{
    if (simulated) {
        ui->statusBox->setText(tr("Please reset!"));
        renderArea->update();
        return;
    }
    simulated = true;
    // get step size and max iterations from GUI.
    rrtstar->setMaxIterations(ui->maxIterations->text().toInt());
    rrtstar->setStepSize(ui->stepSize->text().toInt());

    assert(rrtstar->step_size > 0);
    assert(rrtstar->max_iter > 0);

    // RRTSTAR Algorithm
    for(int i = 0; i < renderArea->rrtstar->max_iter; i++) {
        Node *q = rrtstar->getRandomNode();
        if (q) {
            Node *qNearest = rrtstar->nearest(q->position);
            if (rrtstar->distance(q->position, qNearest->position) > rrtstar->step_size) {
                Vector2f newConfig = rrtstar->newConfig(q, qNearest);
                if (!rrtstar->obstacles->isSegmentInObstacle(newConfig, qNearest->position)) {
                    Node *qNew = new Node;
                    qNew->position = newConfig;

                    vector<Node *> Qnear;
                    rrtstar->near(qNew->position, rrtstar->step_size*3, Qnear);
                    qDebug() << "Found Nearby " << Qnear.size() << "\n";
                    Node *qMin = qNearest;
                    double cmin = rrtstar->Cost(qNearest) + rrtstar->PathCost(qNearest, qNew);
                    for(int j = 0; j < Qnear.size(); j++){
                        Node *qNear = Qnear[j];
                        if(!rrtstar->obstacles->isSegmentInObstacle(qNear->position, qNew->position) &&
                                (rrtstar->Cost(qNear)+rrtstar->PathCost(qNear, qNew)) < cmin ){
                            qMin = qNear; cmin = rrtstar->Cost(qNear)+rrtstar->PathCost(qNear, qNew);
                        }
                    }
                    rrtstar->add(qMin, qNew);

                    for(int j = 0; j < Qnear.size(); j++){
                        Node *qNear = Qnear[j];
                        if(!rrtstar->obstacles->isSegmentInObstacle(qNew->position, qNear->position) &&
                                (rrtstar->Cost(qNew)+rrtstar->PathCost(qNew, qNear)) < rrtstar->Cost(qNear) ){
                            Node *qParent = qNear->parent;
                            // Remove edge between qParent and qNear
                            qParent->children.erase(std::remove(qParent->children.begin(), qParent->children.end(), qNear), qParent->children.end());

                            // Add edge between qNew and qNear
                            qNear->cost = rrtstar->Cost(qNew) + rrtstar->PathCost(qNew, qNear);
                            qNear->parent = qNew;
                            qNew->children.push_back(qNear);
                        }
                    }

                }
            }
        }
        if (rrtstar->reached()) {
            ui->statusBox->setText(tr("Reached Destination!"));
            break;
        }
        renderArea->update();
        qApp->processEvents();
    }
    Node *q;
    if (rrtstar->reached()) {
        q = rrtstar->lastNode;
    }
    else
    {
        // if not reached yet, then shortestPath will start from the closest node to end point.
        q = rrtstar->nearest(rrtstar->endPos);
        ui->statusBox->setText(tr("Exceeded max iterations!"));
    }
    // generate shortest path to destination.
    while (q != NULL) {
        rrtstar->path.push_back(q);
        q = q->parent;
    }
    renderArea->update();
}

/**
 * @brief Delete all obstacles, nodes and paths from the simulator.
 */
void MainWindow::on_resetButton_clicked()
{
    simulated = false;
    ui->statusBox->setText(tr(""));
    rrtstar->obstacles->obstacles.clear();
    rrtstar->obstacles->obstacles.resize(0);
    rrtstar->deleteNodes(rrtstar->root);
    rrtstar->nodes.clear();
    rrtstar->nodes.resize(0);
    rrtstar->path.clear();
    rrtstar->path.resize(0);
    rrtstar->initialize();
    renderArea->update();
}

MainWindow::~MainWindow()
{
    delete ui;
}
