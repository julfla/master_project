""" Forms definitions of pointcloud app. """

from django import forms


class UploadPcdFileForm(forms.Form):

    """ Upload form for a pcd file. """

    pcd_file = forms.FileField()
