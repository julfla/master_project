# Define your item pipelines here
#
# Don't forget to add your pipeline to the ITEM_PIPELINES setting
# See: http://doc.scrapy.org/en/latest/topics/item-pipeline.html

from testapp.models import SketchupModel
from scrapy.exceptions import DropItem
import urllib2 # Use to download objects
 
class SketchupModelPipeline(object):
 
    def process_item(self, item, spider):
        if SketchupModel.objects.filter(google_id=item['google_id']):
       	    raise DropItem("Model %s already in the database." % item['google_id'])
        else:
            item['image'] = urllib2.urlopen(item['link_image']).read()
            item['mesh'] = urllib2.urlopen(item['link_skp']).read()
            item.save()
            return item 
