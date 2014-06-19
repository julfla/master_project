""" Urls definitions of pointcloud app. """

from django.conf.urls import patterns, url

urlpatterns = patterns(
    '',
    url(r'upload_pcd/', 'pointcloud.views.upload_pcd_file')
)
