from django.core.management.base import BaseCommand, CommandError
from warehouse_scrapper.models import WarehouseScrapper
from sketchup_models.models import SketchupModel

class Command(BaseCommand):
    help = 'Does some magical work'

    def handle(self, *args, **options):
        """ Do your work here """
        for keyword in ["banana", "bowl"]:
        	for model in WarehouseScrapper.search_for_models(keyword):
        		model.mesh
        		model.image