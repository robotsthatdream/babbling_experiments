#ifndef DATA_GMM_HPP
#define DATA_GMM_HPP

#include <cafer_core/cafer_core.hpp>
#include <dream_babbling/gmm_archive.h>

class DataGMM : public cafer_core::Data {
    using cafer_core::Data::Data;

    virtual std::map<std::string, std::string> get_serialized_data() const override
    {
        std::map<std::string, std::string> serialized_data;

        cafer_core::shared_ptr<dream_babbling::gmm_archive> gmm_msg;
        gmm_msg = _stored_msg.instantiate<dream_babbling::gmm_archive>();

        serialized_data.emplace("gmm_archive_"+gmm_msg->type.data,gmm_msg->archive.data);

        return serialized_data;
    }
};

#endif //DTA_GMM_HPP
