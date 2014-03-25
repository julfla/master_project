from singleton_models.models import SingletonModel
from django.db import models

class Identifier(SingletonModel):

    categories = models.TextField()
    
    @staticmethod
    def instance():
    	try:
    		return Identifier.objects.all()[0]
    	except:
    		return Identifier()
    
    def identify(self, pointcloud):
        """
        Returns the category of the pointcloud object if its category is known.
        If not, throws an exception.
        """
        raise "Identification failed."

    def train(self, model, category):
        """
        Incrementaly trains the classifier with the new model.
        If the category is not known yet, then it is added and returns true.
        """
        return True
