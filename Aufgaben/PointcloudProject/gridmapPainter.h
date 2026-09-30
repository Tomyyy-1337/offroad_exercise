#include <functional>
#include <optional>
#include <tuple>
#include "rrlib/aspect_maps/tGridAspectMap.h"
#include "rrlib/canvas/tCanvas2D.h"
#include "rrlib/coviroa/color_spaces/tRGB.h"

// Applie up to N coloring rules to a grid map and draw it on a canvas.
template<std::size_t N>
class GridmapPainter
{
public:
  GridmapPainter() :
    draw_functions{}, number_of_custom_rules(0) {}

  // Add a custom coloring rule
  // The rule should return a color for a given value or std::nullopt
  // if the value should not be colored.
  void custom_rule(
    const std::function<std::optional<rrlib::coviroa::color_spaces::tRGB>(float)> &color_fn)
  {
    if (number_of_custom_rules < static_cast<int>(N))
      draw_functions[number_of_custom_rules++] = color_fn;
    else
      throw std::runtime_error("Maximum number of custom rules exceeded");
  }

  // Color cells based on a boolean condition.
  // Paints cells that satisfy the condition with the specified color.
  void conditional_colloring(
    const std::function<bool(float)> &condition,
    const rrlib::coviroa::color_spaces::tRGB &color)
  {
    auto rule = [condition, color](float value) -> std::optional<rrlib::coviroa::color_spaces::tRGB>
    {
      if (condition(value))
        return color;
      else
        return std::nullopt;
    };
    custom_rule(rule);
  }

  // Color cells containing the specified value with the specified color.
  // Other cells will remain unchanged.
  void value_coloring(
    float value,
    const rrlib::coviroa::color_spaces::tRGB &color)
  {
    auto rule = [color, value](float v) -> std::optional<rrlib::coviroa::color_spaces::tRGB>
    {
      if (v == value)
        return color;
      else
        return std::nullopt;
    };
    custom_rule(rule);
  }

  // Transition linearly from one color to another based on the value range.
  // Values below range_start will be colored with start_color,
  // and values above range_end will be colored with end_color.
  void color_range(
    const rrlib::coviroa::color_spaces::tRGB &start_color,
    const rrlib::coviroa::color_spaces::tRGB &end_color,
    float range_start,
    float range_end)
  {
    auto rule = [start_color, end_color, range_start, range_end](float value) -> std::optional<rrlib::coviroa::color_spaces::tRGB>
    {
      if (value <= range_start) return start_color;
      if (value >= range_end) return end_color;

      float ratio = (value - range_start) / (range_end - range_start);
      return rrlib::coviroa::color_spaces::tRGB(
        start_color.R() + ratio * (end_color.R() - start_color.R()),
        start_color.G() + ratio * (end_color.G() - start_color.G()),
        start_color.B() + ratio * (end_color.B() - start_color.B()));
    };
    custom_rule(rule);
  }

  // Transition linearly from one color to another based on the value range,
  // Values outside the range will not be colored.
  void color_range_exclusive(
    const rrlib::coviroa::color_spaces::tRGB &start_color,
    const rrlib::coviroa::color_spaces::tRGB &end_color,
    float range_start,
    float range_end)
  {
    auto rule = [start_color, end_color, range_start, range_end](float value) -> std::optional<rrlib::coviroa::color_spaces::tRGB>
    {
      if (value < range_start || value > range_end) return std::nullopt;

      float ratio = (value - range_start) / (range_end - range_start);
      return rrlib::coviroa::color_spaces::tRGB(
        start_color.R() + ratio * (end_color.R() - start_color.R()),
        start_color.G() + ratio * (end_color.G() - start_color.G()),
        start_color.B() + ratio * (end_color.B() - start_color.B()));
    };
    custom_rule(rule);
  }

  // Draw a grid map to a canvas using the specified coloring rules.
  void draw(
    rrlib::canvas::tCanvas2D &canvas_2d,
    rrlib::aspect_maps::tGridAspectMap<float> const &grid_map) const
  {
    canvas_2d.SetFill(true);

    int start_x = grid_map.GetGridLowerBounds().X();
    int end_x = grid_map.GetGridUpperBounds().X();
    int start_y = grid_map.GetGridLowerBounds().Y();
    int end_y = grid_map.GetGridUpperBounds().Y();

    recursive_draw(start_x, end_x, start_y, end_y, canvas_2d, grid_map);
  }

  void set_downsample_factor(int factor)
  {
    if (factor < 1)
      throw std::invalid_argument("Downsample factor must be at least 1");
    downsample_factor = factor;
  }

  void draw_section(
    rrlib::canvas::tCanvas2D &canvas_2d,
    rrlib::aspect_maps::tGridAspectMap<float> const &grid_map,
    float center_x, float center_y,
    float width, float height) const
  {
    canvas_2d.SetFill(true);

    int x_start = static_cast<int>((center_x - width / 2.0f) / grid_map.GetCellSize().Value()) - 10;
    int x_end = static_cast<int>((center_x + width / 2.0f) / grid_map.GetCellSize().Value()) + 10;
    int y_start = static_cast<int>((center_y - height / 2.0f) / grid_map.GetCellSize().Value()) - 10;
    int y_end = static_cast<int>((center_y + height / 2.0f) / grid_map.GetCellSize().Value()) + 10;
    x_start = std::max(x_start, grid_map.GetGridLowerBounds().X());
    x_end = std::min(x_end, grid_map.GetGridUpperBounds().X());
    y_start = std::max(y_start, grid_map.GetGridLowerBounds().Y());
    y_end = std::min(y_end, grid_map.GetGridUpperBounds().Y());

    recursive_draw(x_start, x_end, y_start, y_end, canvas_2d, grid_map);
  }

private:
  std::array<std::function<std::optional<rrlib::coviroa::color_spaces::tRGB>(float)>, N> draw_functions;
  int number_of_custom_rules = 0;
  int downsample_factor = 1;

  void recursive_draw(
    int x_start, int x_end,
    int y_start, int y_end,
    rrlib::canvas::tCanvas2D &canvas_2d,
    rrlib::aspect_maps::tGridAspectMap<float> const &grid_map) const
  {
    int x_diff = x_end - x_start;
    int y_diff = y_end - y_start;

    if (x_diff <= downsample_factor && y_diff <= downsample_factor)
    {
      auto color = average_color(x_start, x_end, y_start, y_end, grid_map);
      if (color)
      {
        float cell_size = grid_map.GetCellSize().Value();
        canvas_2d.SetColor(*color);
        canvas_2d.DrawBox(
          (x_start - 0.5f) * cell_size,
          (y_start - 0.5f) * cell_size,
          (x_end - x_start) * cell_size,
          (y_end - y_start) * cell_size);
      }
      return;
    }

    auto [fill_color, all_equal] = all_cells_equal(x_start, x_end, y_start, y_end, grid_map);
    if (all_equal)
    {
      if (fill_color)
      {
        float cell_size = grid_map.GetCellSize().Value();
        canvas_2d.SetColor(*fill_color);
        canvas_2d.DrawBox(
          (x_start - 0.5f) * cell_size,
          (y_start - 0.5f) * cell_size,
          (x_end - x_start) * cell_size,
          (y_end - y_start) * cell_size);
      }
      return;
    }

    int x_mid = (x_start + x_end) / 2;
    int y_mid = (y_start + y_end) / 2;

    if (x_diff <= downsample_factor)
    {
      recursive_draw(x_start, x_end, y_start, y_mid, canvas_2d, grid_map);
      recursive_draw(x_start, x_end, y_mid, y_end, canvas_2d, grid_map);
    }
    else if (y_diff <= downsample_factor)
    {
      recursive_draw(x_start, x_mid, y_start, y_end, canvas_2d, grid_map);
      recursive_draw(x_mid, x_end, y_start, y_end, canvas_2d, grid_map);
    }
    else
    {
      recursive_draw(x_start, x_mid, y_start, y_mid, canvas_2d, grid_map);
      recursive_draw(x_mid, x_end, y_start, y_mid, canvas_2d, grid_map);
      recursive_draw(x_start, x_mid, y_mid, y_end, canvas_2d, grid_map);
      recursive_draw(x_mid, x_end, y_mid, y_end, canvas_2d, grid_map);
    }
  }

  std::tuple<std::optional<rrlib::coviroa::color_spaces::tRGB>, bool> all_cells_equal(
    int x_start, int x_end,
    int y_start, int y_end,
    rrlib::aspect_maps::tGridAspectMap<float> const &grid_map) const
  {
    auto first_value = get_color(grid_map.GetCellValue(x_start, y_start));
    for (int x = x_start; x < x_end; ++x)
      for (int y = y_start; y < y_end; ++y)
        if (get_color(grid_map.GetCellValue(x, y)) != first_value)
          return std::tuple(std::nullopt, false);
    return std::tuple(first_value, true);
  }

  std::optional<rrlib::coviroa::color_spaces::tRGB> average_color(
    int x_start, int x_end,
    int y_start, int y_end,
    rrlib::aspect_maps::tGridAspectMap<float> const &grid_map) const
  {
    float r = 0.0f, g = 0.0f, b = 0.0f;
    int count = 0;

    for (int x = x_start; x < x_end; ++x)
      for (int y = y_start; y < y_end; ++y)
      {
        auto color = get_color(grid_map.GetCellValue(x, y));
        if (color)
        {
          r += color->R();
          g += color->G();
          b += color->B();
          ++count;
        }
      }

    if (count == 0) return std::nullopt;

    return rrlib::coviroa::color_spaces::tRGB(r / count, g / count, b / count);
  }

  std::optional<rrlib::coviroa::color_spaces::tRGB> get_color(float value) const
  {
    for (int i = number_of_custom_rules - 1; i >= 0; --i)
    {
      auto color = draw_functions[i](value);
      if (color) return color;
    }
    return std::nullopt;
  }
};