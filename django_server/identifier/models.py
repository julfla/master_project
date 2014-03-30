from singleton_models.models import SingletonModel
from django.db import models
from djangotoolbox.fields import ListField

class Identifier(SingletonModel):

    categories = ListField()
    
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
        if len( self.categories ) == 0: raise IndexError("Identifier is empty.")

        # TODO
        # first behaviour so that can be used in integration.
        from random import randint
        if randint(0, 1) == 0:
            raise Exception("Identification failed.")
        else:
            return self.categories[0]

    def train(self, model, category):
        """
        Incrementaly trains the classifier with the new model.
        If the category is not known yet, then it is added and returns true.
        """
        return None
