from django.test import TestCase


class TestSystemEvaluation(TestCase):

    def test_form_AgreeWithIdentificationForm_validation(self):
        """ Field user_identification can be null only if user disagree. """
        from system_evaluation.forms import AgreeWithIdentificationForm
        data = {'user_agreed': True}
        form = AgreeWithIdentificationForm(data=data)
        self.assertTrue(form.is_valid())
        data = {'user_agreed': False}
        form = AgreeWithIdentificationForm(data=data)
        self.assertFalse(form.is_valid())
        data = {'user_agreed': False, 'user_identification': 'choice'}
        form = AgreeWithIdentificationForm(data=data)
        self.assertTrue(form.is_valid())

    def test_session_can_store_big_identifier(self):
        """ Test that a session can contain a large identifier. """
        from .models import EvaluationSession
        from warehouse_scrapper.models import (search_by_keywords,
                                               retreive_model)
        session = EvaluationSession()
        categories = ['dinner plate', 'toothbrush', 'apple fruit', 'soda can']
        for category in categories:
            models = [retreive_model(id) for id in
                      search_by_keywords(category)]
            models = [model for model in models if model is not None]
            session.identifier.add_models(models, category)
        session.identifier.train()
        import sys
        import pickle
        serialized_size = sys.getsizeof(pickle.dumps(session.identifier))
        print "Size needed: ", serialized_size, " bytes."
        self.assertTrue(serialized_size > 16 * 1024 * 1024)  # 16 Mo
        session.save()
