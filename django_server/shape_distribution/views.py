from shape_distribution.models import ShapeDistribution
from django.http import HttpResponse

from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.dates import DateFormatter

def distribution_image(request, distribution):
    fig = Figure()
    ax = fig.add_subplot(111)
    ax.plot( distribution.as_numpy_array )
    canvas = FigureCanvas(fig)
    response = HttpResponse(content_type='image/png')
    canvas.print_png(response)
    return response
    