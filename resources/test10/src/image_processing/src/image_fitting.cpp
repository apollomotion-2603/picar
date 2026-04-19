#include "image_declarations.h"

// ── poly_fit ─────────────────────────────────────────────────────────────────
// Least-squares polynomial fit bậc `degree` qua N điểm (rows[i], centroid_x[i])
// dùng normal equations A^T A c = A^T b, giải bằng Gauss-Jordan elimination.
// Trả về coeffs [c0, c1, c2] sao cho:  x = c0 + c1*y + c2*y^2
std::vector<double> ImageProcessor::poly_fit(
    const std::vector<int>&    rows,
    const std::vector<double>& centroid_x,
    int degree)
{
    int n = (int)rows.size();
    int d = degree + 1;

    // Augmented matrix [A^T A | A^T b] kích thước d x (d+1)
    std::vector<std::vector<double>> M(d, std::vector<double>(d + 1, 0.0));

    for (int i = 0; i < n; i++) {
        double y = static_cast<double>(rows[i]);
        double x = centroid_x[i];

        // Lũy thừa y^0 .. y^(2*degree)
        std::vector<double> yp(2 * d, 1.0);
        for (int k = 1; k < 2 * d; k++) yp[k] = yp[k - 1] * y;

        for (int r = 0; r < d; r++) {
            M[r][d] += yp[r] * x;           // A^T b
            for (int c = 0; c < d; c++)
                M[r][c] += yp[r + c];        // A^T A
        }
    }

    // Gauss-Jordan elimination với partial pivoting
    for (int col = 0; col < d; col++) {
        // Tìm pivot lớn nhất
        int pivot = col;
        for (int row = col + 1; row < d; row++)
            if (std::abs(M[row][col]) > std::abs(M[pivot][col])) pivot = row;
        std::swap(M[col], M[pivot]);

        double div = M[col][col];
        if (std::abs(div) < 1e-10) continue;  // ma trận singular, bỏ qua

        for (int row = 0; row < d; row++) {
            if (row == col) continue;
            double factor = M[row][col] / div;
            for (int k = col; k <= d; k++)
                M[row][k] -= factor * M[col][k];
        }
    }

    std::vector<double> coeffs(d);
    for (int i = 0; i < d; i++)
        coeffs[i] = (std::abs(M[i][i]) > 1e-10) ? M[i][d] / M[i][i] : 0.0;

    return coeffs;  // x = coeffs[0] + coeffs[1]*y + coeffs[2]*y^2
}

// ── draw_poly_curve ───────────────────────────────────────────────────────────
// Evaluate đường cong polynomial từng pixel row rồi tô màu cam
void ImageProcessor::draw_poly_curve(
    unsigned char* d,
    int step, int channels, int width,
    const std::vector<double>& coeffs,
    int scan_top, int scan_bot)
{
    const uint8_t R = 255, G = 140, B = 0;  // màu cam

    for (int y = scan_top; y <= scan_bot; y++) {
        // Evaluate x = c0 + c1*y + c2*y^2
        double x = 0.0, yp = 1.0;
        for (double c : coeffs) { x += c * yp; yp *= y; }

        int xi = static_cast<int>(std::round(x));
        if (xi < 1 || xi >= width - 1) continue;

        // Vẽ chấm rộng 3px ngang để dễ nhìn
        for (int dx = -1; dx <= 1; dx++) {
            int px = y * step + (xi + dx) * channels;
            d[px] = R; d[px + 1] = G; d[px + 2] = B;
        }
    }
}

// ── publish_fitting_image ─────────────────────────────────────────────────────
// Vẽ lên ảnh sạch: 5 centroid (trắng) + đường cong poly fit (cam) + điểm vanishing (đỏ)
void ImageProcessor::publish_fitting_image(
    const sensor_msgs::msg::Image::SharedPtr src,
    const std::vector<int>&    scan_rows,
    const std::vector<double>& centroid_x,
    const std::vector<double>& poly_coeffs,
    int scan_top, int scan_bot)
{
    auto out = clone_image(src);

    const int width    = static_cast<int>(src->width);
    const int step     = static_cast<int>(src->step);
    const int channels = step / width;
    const int height   = static_cast<int>(src->height);
    unsigned char* d   = out->data.data();

    // 1. Vẽ đường cong polynomial (cam)
    draw_poly_curve(d, step, channels, width, poly_coeffs, scan_top, scan_bot);

    // 2. Vẽ 5 centroid điểm dữ liệu (trắng 5x5)
    for (int i = 0; i < N; i++) {
        int ry = scan_rows[i];
        int rx = static_cast<int>(std::round(centroid_x[i]));
        for (int dr = -2; dr <= 2; dr++)
        for (int dc = -2; dc <= 2; dc++) {
            int r = ry + dr, x = rx + dc;
            if (r < 0 || r >= height || x < 0 || x >= width) continue;
            int px = r * step + x * channels;
            d[px] = 255; d[px + 1] = 255; d[px + 2] = 255;
        }
    }

    // 3. Điểm vanishing = giao đường cong với scan_top (đỏ 7x7)
    {
        double x = 0.0, yp = 1.0;
        for (double c : poly_coeffs) { x += c * yp; yp *= scan_top; }
        int vx = static_cast<int>(std::round(x));
        int vy = scan_top;
        for (int dr = -3; dr <= 3; dr++)
        for (int dc = -3; dc <= 3; dc++) {
            int r = vy + dr, xi = vx + dc;
            if (r < 0 || r >= height || xi < 0 || xi >= width) continue;
            int px = r * step + xi * channels;
            d[px] = 255; d[px + 1] = 0; d[px + 2] = 0;  // đỏ
        }
    }

    pub_fitting_image_->publish(*out);
}
