#include "projects/PointcloudProject/mPointcloudToHeightMap.h"
#include "projects/PointcloudProject/gridmapPainter.h"

#include "rrlib/coviroa/color_spaces/tRGB.h"

namespace finroc::PointcloudProject
{

#ifdef _LIB_FINROC_PLUGINS_RUNTIME_CONSTRUCTION_ACTIONS_PRESENT_
static const runtime_construction::tStandardCreateModuleAction<mPointcloudToHeightMap> cCREATE_ACTION_FOR_M_POINTCLOUD_TO_HEIGHT_MAP("PointcloudToHeightMap");
#endif

mPointcloudToHeightMap::mPointcloudToHeightMap(core::tFrameworkElement *parent, const std::string &name,
                                               ib2c::tStimulationMode stimulation_mode) :
  tModule(parent, name, stimulation_mode, false),
  par_cell_size("Cell Size", rrlib::si_units::tLength<float>(0.01f), this),
  par_map_size("Map Size", rrlib::si_units::tLength<float>(0.5f), this)
{
  initialize_height_map();
}

mPointcloudToHeightMap::~mPointcloudToHeightMap()
{}

void mPointcloudToHeightMap::OnParameterChange()
{
  tModule::OnParameterChange();
  initialize_height_map();
}

bool mPointcloudToHeightMap::ProcessTransferFunction()
{
  if (!in_pointcloud.HasChanged())
  {
    return false;
  }

  height_map.SetAllValues(0.0f);

  auto pointcloud = in_pointcloud.GetPointer();
  const auto *points = reinterpret_cast<const rrlib::math::tVec3f *>(pointcloud->DataPtr());
  for (unsigned int index = 0; index < pointcloud->Dimension(); ++index)
  {
    insert_point_in_height_map(points[index]);
  }

  out_height_map.Publish(height_map);
  VisualizeHeightMap();
  return true;
}

ib2c::tTargetRating mPointcloudToHeightMap::CalculateTargetRating() const
{
  return 1.0;
}

void mPointcloudToHeightMap::insert_point_in_height_map(const rrlib::math::tVec3f &point)
{
  const auto cell_size = height_map.GetCellSize();
  const int x_index = static_cast<int>(point.X() / cell_size);
  const int y_index = static_cast<int>(point.Y() / cell_size);
  const float existing_value = height_map.GetCellValue(x_index, y_index);
  const float new_value = point.Z() + 10.0f;

  if (new_value > existing_value)
  {
    height_map.SetCellValue(x_index, y_index, new_value);
  }
}

void mPointcloudToHeightMap::VisualizeHeightMap()
{
  auto canvas = vis_height_map_2D.GetUnusedBuffer();
  canvas->Clear();

  GridmapPainter<2> painter;
  painter.color_range(
    rrlib::coviroa::color_spaces::tRGB(52.0f, 152.0f, 235.0f),
    rrlib::coviroa::color_spaces::cRGB_BLACK,
    10.0f,
    11.0f);
  painter.value_coloring(0.0f, rrlib::coviroa::color_spaces::cRGB_WHITE);
  painter.draw(*canvas, height_map);

  vis_height_map_2D.Publish(canvas);
}

void mPointcloudToHeightMap::initialize_height_map()
{
  height_map = rrlib::aspect_maps::tGridAspectMap<>();
  const std::size_t size = static_cast<std::size_t>(par_map_size.Get().Value() / par_cell_size.Get().Value());
  height_map.SetSize(size, size, par_cell_size.Get().Value());
}

}  // namespace finroc::PointcloudProject