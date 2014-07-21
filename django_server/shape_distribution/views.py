from shape_distribution.models import (ShapeDistribution,
                                       SHAPE_DISTRIBUTION_SIZE)
from django.http import HttpResponse

from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.dates import DateFormatter

def distribution_image(request, distribution):
    fig = Figure()
    ax = fig.add_subplot(111)
    ax.bar( range(SHAPE_DISTRIBUTION_SIZE), distribution.as_numpy_array, width=1)
    canvas = FigureCanvas(fig)
    response = HttpResponse(content_type='image/png')
    canvas.print_png(response)
    return response

