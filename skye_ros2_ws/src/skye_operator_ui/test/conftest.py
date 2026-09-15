import pytest

from fakes import make_fake_stack


@pytest.fixture
def fake_stack():
    return make_fake_stack()
