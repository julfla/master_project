from django import forms

BOOL_CHOICES = ((True, 'Yes'), (False, 'No'))

class NewSessionForm(forms.Form):
    user  = forms.CharField(label='Name',
    	widget=forms.TextInput(attrs={'placeholder': 'Enter your name here...'})
    	)

class AgreeWithIdentificationForm(forms.Form):
    user_argreed = forms.BooleanField(label="Do you agree ?")
    user_identification = forms.CharField(label="My own choice")