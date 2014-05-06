from django.conf.urls import patterns, include, url

urlpatterns = patterns('',
    url(r'^(.+)/plot_distribution', 'partial_view.views.image_plot_distribution')
)
