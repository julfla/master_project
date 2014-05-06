from django import forms

class UploadPcdFileForm(forms.Form):
    pcd_file  = forms.FileField()