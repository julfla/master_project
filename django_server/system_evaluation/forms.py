from django import forms

BOOL_CHOICES = ((True, 'Yes'), (False, 'No'))


class NewSessionForm(forms.Form):

    """ A form to the user for information about him. """

    user = forms.CharField(label='Name', widget=forms.TextInput(
        attrs={'placeholder': 'Enter your name here...'}))


class EndSessionForm(forms.Form):

    """  A form to ask for a global opinion to the user. """

    label = "How many times did the system fail to recognize an object ?"
    number_failures = forms.IntegerField(
        widget=forms.Textarea, label=label, required=False)

    label = "Would you mind having to teach a robot in order to use it afterwards ?"
    agree_teaching = forms.CharField(
        widget=forms.Textarea, label=label, required=False)

    label = "What kind of interface would you want to use when teaching something to a Robot ?"
    other_interface = forms.CharField(
        widget=forms.Textarea, label=label, required=False)

    label = "What would you teach to a robot if it could recognize objects ?"
    action_robot = forms.CharField(
        widget=forms.Textarea, label=label, required=False)


class AgreeWithIdentificationForm(forms.Form):

    """ A form to ask the user his opinion on an identification result. """

    user_agreed = forms.BooleanField(label="Do you agree ?", required=False)
    user_identification = forms.CharField(label="My own choice",
                                          required=False)

    def clean_user_identification(self):
        """ Check if the data form is valid. """
        cleaned_data = self.cleaned_data
        user_agreed = cleaned_data.get("user_agreed")
        user_identification = cleaned_data.get("user_identification")
        if not user_identification and not user_agreed:
            raise forms.ValidationError(
                ("If you disagree with the, ",
                 "identification please give us your personnal choice."))
        else:
            return cleaned_data
