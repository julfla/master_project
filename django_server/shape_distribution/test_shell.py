from sketchup_models.models import SketchupModel
from partial_view.models import PartialView
from shape_distribution.models import ShapeDistribution
from common.libs.libpydescriptors import Distribution

model = SketchupModel.objects.all()[0]
view = PartialView.compute_view( model, 0.0, 0.0 )
dis = ShapeDistribution.compute( view.pointcloud )
