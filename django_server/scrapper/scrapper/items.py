# Define here the models for your scraped items
#
# See documentation in:
# http://doc.scrapy.org/en/latest/topics/items.html

from scrapy.item import Item, Field
from scrapy.contrib.djangoitem import DjangoItem

from testapp.models import SketchupModel

class ModelItem(DjangoItem):
    django_model = SketchupModel
    link_image = Field()
    link_skp = Field()
    link_full_description = Field()
