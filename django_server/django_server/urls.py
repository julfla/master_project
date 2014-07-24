from django.conf.urls import patterns, include, url
from django.views.generic import RedirectView

from django.contrib import admin
admin.autodiscover()

urlpatterns = patterns(
    '',
    url(r'^admin/', include(admin.site.urls)),
    url(r'^pointcloud/', include('pointcloud.urls')),
    url(r'^model/', include('sketchup_models.urls')),
    url(r'^partial/', include('partial_view.urls')),
    url(r'^distribution/', include('shape_distribution.urls')),
    url(r'^system/', include('system_evaluation.urls')),
    url(r'', RedirectView.as_view(url='/system/session/new'))
    )
