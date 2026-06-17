#include "visualizer.h"

using namespace cv;
using namespace std;

// =============================================================================
// 常量
// =============================================================================
namespace {
constexpr size_t kRangePlotSamples = 600;
constexpr int kPlotW = 960;
constexpr int kPlotH = 420;
} // namespace

// =============================================================================
// 数据管理
// =============================================================================
void pushRangeSampleRaw(std::deque<float> &q, float v)
{
    q.push_back(v);
    while (q.size() > kRangePlotSamples)
        q.pop_front();
}

// =============================================================================
// 内部辅助
// =============================================================================
namespace {

void autoYAxisFinite(const std::deque<float> &data, float &ymin, float &ymax)
{
    bool any = false;
    float lo = 0.f, hi = 0.f;
    for (float v : data)
    {
        if (!std::isfinite(v))
            continue;
        if (!any)
        {
            lo = hi = v;
            any = true;
        }
        else
        {
            lo = std::min(lo, v);
            hi = std::max(hi, v);
        }
    }
    if (!any)
    {
        ymin = 0.f;
        ymax = 5.f;
        return;
    }
    ymin = lo;
    ymax = hi;
    float span = ymax - ymin;
    float pad = std::max(0.08f, span * 0.12f);
    ymin -= pad;
    ymax += pad;
    if (ymax - ymin < 0.2f)
    {
        ymin -= 0.1f;
        ymax += 0.1f;
    }
}

void autoYAxisFiniteShared(const std::deque<float> &a, const std::deque<float> &b,
                           float &ymin, float &ymax)
{
    bool any = false;
    float lo = 0.f, hi = 0.f;
    auto consider = [&](float v) {
        if (!std::isfinite(v))
            return;
        if (!any)
        {
            lo = hi = v;
            any = true;
        }
        else
        {
            lo = std::min(lo, v);
            hi = std::max(hi, v);
        }
    };
    for (float v : a)
        consider(v);
    for (float v : b)
        consider(v);
    if (!any)
    {
        ymin = 0.f;
        ymax = 5.f;
        return;
    }
    ymin = lo;
    ymax = hi;
    float span = ymax - ymin;
    float pad = std::max(0.08f, span * 0.12f);
    ymin -= pad;
    ymax += pad;
    if (ymax - ymin < 0.2f)
    {
        ymin -= 0.1f;
        ymax += 0.1f;
    }
}

void drawFinitePolyline(Mat &canvas, int ox, int oy_bot, int w, int h, float ymin, float ymax,
                        const std::deque<float> &data, const Scalar &line_color,
                        int n_plot = -1)
{
    std::vector<Point> seg;
    const int n = n_plot > 0 ? std::min(n_plot, static_cast<int>(data.size()))
                             : static_cast<int>(data.size());
    for (int i = 0; i < n; ++i)
    {
        float v = data[static_cast<size_t>(i)];
        if (!std::isfinite(v))
        {
            if (seg.size() >= 2)
                polylines(canvas, seg, false, line_color, 2, LINE_AA);
            seg.clear();
            continue;
        }
        int px = ox + i * w / std::max(1, n - 1);
        int py = oy_bot - cvRound((v - ymin) / (ymax - ymin + 1e-6f) * h);
        seg.push_back(Point(px, py));
    }
    if (seg.size() >= 2)
        polylines(canvas, seg, false, line_color, 2, LINE_AA);
}

void drawRangeStrip(Mat &canvas, const Rect &band, const std::deque<float> &data,
                    const char *title, const Scalar &line_color)
{
    rectangle(canvas, band, Scalar(32, 32, 32), FILLED);
    putText(canvas, title, Point(band.x + 8, band.y + 22),
            FONT_HERSHEY_SIMPLEX, 0.55, Scalar(220, 220, 220), 1, LINE_AA);

    float ymin = 0, ymax = 5;
    autoYAxisFinite(data, ymin, ymax);

    const int margin_l = 52;
    const int margin_r = 10;
    const int margin_t = 30;
    const int margin_b = 12;
    int ox = band.x + margin_l;
    int oy_top = band.y + margin_t;
    int w = band.width - margin_l - margin_r;
    int h = band.height - margin_t - margin_b;
    int oy_bot = oy_top + h;

    if (w < 4 || h < 4 || data.size() < 2)
    {
        putText(canvas, "(waiting samples...)", Point(ox, oy_top + h / 2),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(140, 140, 140), 1, LINE_AA);
        return;
    }

    for (int gi = 0; gi <= 4; ++gi)
    {
        float g = ymin + (ymax - ymin) * (gi / 4.f);
        int gy = oy_bot - cvRound((g - ymin) / (ymax - ymin + 1e-6f) * h);
        line(canvas, Point(ox, gy), Point(ox + w, gy), Scalar(55, 55, 55), 1, LINE_AA);
        putText(canvas, format("%.2f", g), Point(band.x + 4, gy + 4),
                FONT_HERSHEY_SIMPLEX, 0.35, Scalar(110, 110, 110), 1, LINE_AA);
    }

    drawFinitePolyline(canvas, ox, oy_bot, w, h, ymin, ymax, data, line_color, -1);

    std::string cur_txt = "now: ";
    if (!data.empty() && std::isfinite(data.back()))
        cur_txt += format("%.4f", data.back());
    else if (!data.empty())
        cur_txt += "non-finite";
    else
        cur_txt += "---";
    putText(canvas, cur_txt, Point(ox + w - 240, band.y + 22),
            FONT_HERSHEY_SIMPLEX, 0.5, line_color, 1, LINE_AA);
}

void drawRangeStripYzShared(Mat &canvas, const Rect &band, const std::deque<float> &qy,
                            const std::deque<float> &qz)
{
    rectangle(canvas, band, Scalar(32, 32, 32), FILLED);
    putText(canvas, "ob_y + ob_z (shared Y, raw)", Point(band.x + 8, band.y + 22),
            FONT_HERSHEY_SIMPLEX, 0.55, Scalar(220, 220, 220), 1, LINE_AA);

    float ymin = 0, ymax = 5;
    autoYAxisFiniteShared(qy, qz, ymin, ymax);

    const int margin_l = 52;
    const int margin_r = 10;
    const int margin_t = 30;
    const int margin_b = 12;
    int ox = band.x + margin_l;
    int oy_top = band.y + margin_t;
    int w = band.width - margin_l - margin_r;
    int h = band.height - margin_t - margin_b;
    int oy_bot = oy_top + h;

    const int n = static_cast<int>(std::min(qy.size(), qz.size()));
    if (w < 4 || h < 4 || n < 2)
    {
        putText(canvas, "(waiting samples...)", Point(ox, oy_top + h / 2),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(140, 140, 140), 1, LINE_AA);
        return;
    }

    for (int gi = 0; gi <= 4; ++gi)
    {
        float g = ymin + (ymax - ymin) * (gi / 4.f);
        int gy = oy_bot - cvRound((g - ymin) / (ymax - ymin + 1e-6f) * h);
        line(canvas, Point(ox, gy), Point(ox + w, gy), Scalar(55, 55, 55), 1, LINE_AA);
        putText(canvas, format("%.2f", g), Point(band.x + 4, gy + 4),
                FONT_HERSHEY_SIMPLEX, 0.35, Scalar(110, 110, 110), 1, LINE_AA);
    }

    drawFinitePolyline(canvas, ox, oy_bot, w, h, ymin, ymax, qy, Scalar(80, 255, 140), n);
    drawFinitePolyline(canvas, ox, oy_bot, w, h, ymin, ymax, qz, Scalar(200, 120, 255), n);

    putText(canvas, "ob_y", Point(ox + w - 220, band.y + 22),
            FONT_HERSHEY_SIMPLEX, 0.45, Scalar(80, 255, 140), 1, LINE_AA);
    putText(canvas, "ob_z", Point(ox + w - 120, band.y + 22),
            FONT_HERSHEY_SIMPLEX, 0.45, Scalar(200, 120, 255), 1, LINE_AA);
}

} // namespace

// =============================================================================
// 公开接口
// =============================================================================
Mat makeRangePlotMat(const std::deque<float> &qx,
                     const std::deque<float> &qy,
                     const std::deque<float> &qz)
{
    Mat canvas(kPlotH, kPlotW, CV_8UC3, Scalar(20, 20, 20));
    int bh0 = kPlotH / 2;
    int bh1 = kPlotH - bh0;
    drawRangeStrip(canvas, Rect(0, 0, kPlotW, bh0), qx, "ob_x (raw)", Scalar(80, 180, 255));
    drawRangeStripYzShared(canvas, Rect(0, bh0, kPlotW, bh1), qy, qz);
    return canvas;
}