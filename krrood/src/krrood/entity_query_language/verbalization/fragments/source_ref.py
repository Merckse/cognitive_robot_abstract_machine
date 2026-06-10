from __future__ import annotations

from dataclasses import dataclass
from typing_extensions import Optional


@dataclass(frozen=True)
class SourceRef:
    """
    Carries a reference to the Python source entity that a
    :class:`~krrood.entity_query_language.verbalization.fragments.base.RoleFragment` represents.

    Used by :class:`~krrood.entity_query_language.verbalization.rendering.source_link_resolver.SourceLinkResolver`
    implementations to resolve the fragment to a documentation URL.

    Frozen (immutable) so instances can be safely shared and hashed.
    """

    owner_type: type
    """The Python class this fragment refers to (always set)."""

    attribute: Optional[str] = None
    """Attribute name within *owner_type*; ``None`` means the fragment refers
    to the class itself (e.g. for type-name labels like *"Robot"*)."""

    @classmethod
    def for_type(cls, t) -> Optional[SourceRef]:
        """
        Return ``SourceRef(owner_type=t)`` when *t* is a real ``type``, else ``None``.

        :param t: Candidate type (any value accepted; non-types return ``None``).
        :return: A :class:`SourceRef` for the class, or ``None``.
        :rtype: SourceRef or None
        """
        return cls(owner_type=t) if isinstance(t, type) else None

    @classmethod
    def for_attribute(cls, owner, attribute_name: str) -> Optional[SourceRef]:
        """
        Return ``SourceRef(owner_type=owner, attribute=attribute_name)`` when *owner* is a real ``type``,
        else ``None``.

        :param owner: Candidate owner class (any value; non-types return ``None``).
        :param attribute_name: Canonical attribute name on *owner*.
        :type attribute_name: str
        :return: A :class:`SourceRef` for the attribute, or ``None``.
        :rtype: SourceRef or None
        """
        return (
            cls(owner_type=owner, attribute=attribute_name)
            if isinstance(owner, type)
            else None
        )
