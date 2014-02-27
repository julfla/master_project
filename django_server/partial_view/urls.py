from django.conf.urls import patterns, include, url

urlpatterns = patterns('',
    url(r'^(.+)/pcd/', 'partial_view.views.view_pcd')
)
