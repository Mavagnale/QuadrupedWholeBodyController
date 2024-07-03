#include "sliderGui.h"

SliderGUI::SliderGUI(int num_sliders, QWidget *parent)
    : QWidget(parent)
{
    QVBoxLayout *layout = new QVBoxLayout(this);

    for (int i = 0; i < num_sliders; ++i) {
        QSlider *slider = new QSlider(Qt::Horizontal);
        slider->setRange(-1000, 1000);
        sliders.push_back(slider);
        layout->addWidget(slider);

        QLabel *label = new QLabel("0");
        labels.push_back(label);
        layout->addWidget(label);

        connect(slider, &QSlider::valueChanged, [this, i]() {
            labels[i]->setText(QString::number(sliders[i]->value() / 1000.0 ));
        });
    }
}

std::vector<double> SliderGUI::getSliderValues() const
{
    std::vector<double> values;
    for (const auto& slider : sliders) {
        values.push_back(slider->value());
    }
    return values;
}
