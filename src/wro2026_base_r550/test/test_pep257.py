"""ament_pep257 (only where ament is installed, e.g. the Jetson)."""
import pytest

ament_pep257 = pytest.importorskip('ament_pep257.main')


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = ament_pep257.main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
