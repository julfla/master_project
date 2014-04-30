from django.http import HttpResponse, HttpResponseNotFound
from django.shortcuts import get_object_or_404

from partial_view.models import PartialView

def image_plot_distribution(request, partial_view_id):
    view = get_object_or_404(PartialView, pk=partial_view_id)
    from shape_distribution.views import distribution_image
    return distribution_image(request, view.distribution)




