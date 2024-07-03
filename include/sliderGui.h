#ifndef SLIDER_GUI_H
#define SLIDER_GUI_H

#include <QWidget>
#include <QSlider>
#include <QVBoxLayout>
#include <QLabel>
#include <vector>

class SliderGUI : public QWidget
{
public:
    SliderGUI(int num_sliders, QWidget *parent = 0);
    std::vector<double> getSliderValues() const;

private:
    std::vector<QSlider*> sliders;
    std::vector<QLabel*> labels;
};

#endif // SLIDER_GUI_H
