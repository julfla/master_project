""" Views definitions of pointcloud app. """

from django.shortcuts import render_to_response, render
from .forms import UploadPcdFileForm


def upload_pcd_file(request):
    """ Upload view. """
    if request.method == 'POST':
        form = UploadPcdFileForm(request.POST, request.FILES)
        if form.is_valid():
            return render_to_response(
                'base_html.html',
                {'content_html': request.FILES['pcd_file'].read()})
    else:
        form = UploadPcdFileForm()
    return render(request, 'upload_pcd_form.html', {'form': form})
