from django.conf.urls import patterns, include, url

urlpatterns = patterns('',
    url(r'upload_pcd/', 'pointcloud.views.upload_pcd_file')
)
