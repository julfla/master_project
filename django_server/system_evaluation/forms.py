from django import forms

BOOL_CHOICES = ((True, 'Yes'), (False, 'No'))

class NewSessionForm(forms.Form):
    user  = forms.CharField(label='Name',
    	widget=forms.TextInput(attrs={'placeholder': 'Enter your name here...'})
    	)

class AgreeWithIdentificationForm(forms.Form):
    user_agreed = forms.BooleanField(label="Do you agree ?", required=False)
    user_identification = forms.CharField(label="My own choice", required=False)

    def clean_user_identification(self):
        cleaned_data = self.cleaned_data
        user_agreed = cleaned_data.get("user_agreed")
        user_identification = cleaned_data.get("user_identification")
        if not user_identification and not user_agreed:
            raise forms.ValidationError(
              	"If you disagree with the identification please give us your choice.")
        else:
            return cleaned_data