/*
 * Copyright (c) 2019-2020, Jesse Buhagiar <jooster669@gmail.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include "MonitorWidget.h"
#include <LibCore/Timer.h>
#include <LibEDID/EDID.h>
#include <LibGfx/FontDatabase.h>
#include <LibGUI/ColorInput.h>
#include <LibGUI/ComboBox.h>
#include <LibGUI/ItemListModel.h>
#include <LibGUI/RadioButton.h>
#include <LibGUI/SettingsWindow.h>
#include <WindowServer/ScreenLayout.h>

namespace DisplaySettings {

class MonitorSettingsWidget final : public GUI::SettingsWindow::Tab {
    C_OBJECT(MonitorSettingsWidget);

public:
    ~MonitorSettingsWidget() override
    {
        if (m_showing_screen_numbers)
            show_screen_numbers(false);
    }

    virtual void apply_settings() override;
    void show_screen_numbers(bool);

protected:
    void show_event(GUI::ShowEvent& event) override;
    void hide_event(GUI::HideEvent& event) override;

private:
    MonitorSettingsWidget();

    void create_frame();
    void create_default_resolution_list();
    bool create_resultion_list_from_edid();
    void load_current_settings();
    void selected_screen_index_or_resolution_changed();

    size_t m_selected_screen_index { 0 };

    WindowServer::ScreenLayout m_screen_layout;
    Vector<String> m_screens;
    Vector<Optional<EDID::Parser>> m_screen_edids;

    using ResolutionInfo = EDID::Parser::SupportedResolution;

    class ResolutionInfoModel final : public GUI::Model {
    public:
        template<typename... Args>
        static NonnullRefPtr<ResolutionInfoModel> create(Args&&... args)
        {
            return adopt_ref(*new ResolutionInfoModel(forward<Args>(args)...));
        }
        virtual ~ResolutionInfoModel() override = default;

        virtual int row_count(GUI::ModelIndex const&) const override { return m_data.size(); }
        virtual int column_count(GUI::ModelIndex const&) const override { return 1; }
        virtual String column_name(int) const override { return "Resolution"sv; }
        virtual bool is_searchable() const override { return false; }
        virtual GUI::Variant data(GUI::ModelIndex const& index, GUI::ModelRole role) const override
        {
            switch (role) {
            case GUI::ModelRole::TextAlignment:
                return Gfx::TextAlignment::CenterLeft;
            case GUI::ModelRole::Display: {
                auto& resolution_info = m_data[(size_t)index.row()];
                return String::formatted("{} x {}", resolution_info.width, resolution_info.height);
            }
            case GUI::ModelRole::Custom:
                return index.row();
            default:
                break;
            }
            return {};
        }

    private:
        explicit ResolutionInfoModel(Vector<ResolutionInfo> const& data)
            : m_data(data)
        {
        }
        Vector<ResolutionInfo> const& m_data;
    };
    Vector<ResolutionInfo> m_resolutions;
    Vector<int, 4> m_selected_resolution_refresh_rates;

    RefPtr<DisplaySettings::MonitorWidget> m_monitor_widget;
    RefPtr<GUI::ComboBox> m_screen_combo;
    RefPtr<GUI::ComboBox> m_resolution_combo;
    RefPtr<GUI::ComboBox> m_refresh_rate_combo;
    RefPtr<GUI::RadioButton> m_display_scale_radio_1x;
    RefPtr<GUI::RadioButton> m_display_scale_radio_2x;
    RefPtr<GUI::Label> m_dpi_label;

    bool m_showing_screen_numbers { false };
};

}
